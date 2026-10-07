#include "pb_nav_goal_decision/goal_decision_node.hpp"

#include <chrono>
#include <fstream>
#include <stdexcept>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "action_msgs/msg/goal_status.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "yaml-cpp/yaml.h"

namespace pb_nav_goal_decision
{
namespace
{
// Single source of truth for the Nav2 action name: the action client and the
// `.../_action/status` subscription below must always agree on it.
constexpr char kActionName[] = "navigate_to_pose";

// After issuing a timeout cancel, wait this long for the resulting CANCELED
// result. If it never arrives (broken/silent server) the goal is written off
// locally so that the retry logic still runs instead of hanging forever.
constexpr double kTimeoutCancelGraceSec = 5.0;
}  // namespace

GoalDecisionNode::GoalDecisionNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("goal_decision_node", options),
  game_start_progress_threshold_(4),
  hp_return_threshold_(30),
  goal_timeout_sec_(60.0),
  max_retries_(2),
  retry_delay_sec_(3.0),
  use_arbitration_(true),
  arbitration_resume_delay_sec_(1.0),
  arbitration_status_stale_sec_(3.0),
  check_localization_(true),
  localization_map_frame_("map"),
  localization_odom_frame_("odom"),
  state_(State::WAIT_GAME_START),
  game_started_(false),
  hp_low_(false),
  current_goal_name_(""),
  goal_active_(false),
  server_ready_(false)
{
  // ---- load config file ----
  std::string config_file = this->declare_parameter<std::string>("config_file", "");
  if (config_file.empty()) {
    try {
      config_file = ament_index_cpp::get_package_share_directory("pb_nav_goal_decision") +
                    "/config/goal_decision_params.yaml";
    } catch (const std::exception & e) {
      RCLCPP_WARN(get_logger(), "Could not resolve default config path: %s", e.what());
    }
  }
  if (config_file.empty()) {
    RCLCPP_WARN(get_logger(), "Config file path is empty; using built-in defaults.");
  } else if (!std::ifstream(config_file)) {
    RCLCPP_WARN(get_logger(), "Config file not found: '%s' (using built-in defaults).",
                config_file.c_str());
  } else {
    loadConfig(config_file);
  }

  // ---- subscriptions (referee data, relayed by the forwarding/serial node as std_msgs) ----
  game_progress_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
    "referee/game_progress", 10,
    std::bind(&GoalDecisionNode::gameProgressCallback, this, std::placeholders::_1));
  current_hp_sub_ = this->create_subscription<std_msgs::msg::Int32>(
    "referee/current_hp", 10,
    std::bind(&GoalDecisionNode::currentHpCallback, this, std::placeholders::_1));
  maximum_hp_sub_ = this->create_subscription<std_msgs::msg::Int32>(
    "referee/maximum_hp", 10,
    std::bind(&GoalDecisionNode::maximumHpCallback, this, std::placeholders::_1));

  // ---- action client to Nav2 bt_navigator ----
  action_client_ = rclcpp_action::create_client<NavigateToPose>(this, kActionName);

  // ---- arbitration: watch every goal on that action, whoever sent it ----
  // A ROS action offers no API to enumerate other clients' goals, but the
  // action server republishes the status of ALL of them on its own hidden
  // status topic. We use it to detect manual goals (RViz "Nav2 Goal" tool,
  // joystick in auto_control mode) and stand by while they are active.
  if (use_arbitration_) {
    goal_status_sub_ = this->create_subscription<action_msgs::msg::GoalStatusArray>(
      std::string(kActionName) + "/_action/status", 10,
      std::bind(&GoalDecisionNode::goalStatusCallback, this, std::placeholders::_1));
  }

  // ---- localization readiness: wait for `map -> odom` to appear on tf ----
  // Relative topic names on purpose: they resolve to <namespace>/tf(_static),
  // which is exactly where the relocalization node broadcasts (the nav2 stack
  // remaps /tf -> tf inside its namespace).
  if (check_localization_) {
    tf_sub_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
      "tf", 100, std::bind(&GoalDecisionNode::tfCallback, this, std::placeholders::_1));
    tf_static_sub_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
      "tf_static", 100, std::bind(&GoalDecisionNode::tfCallback, this, std::placeholders::_1));
  }

  // ---- watchdog: server / localization / timeout / retry / resume ----
  watchdog_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&GoalDecisionNode::watchdogTimerCallback, this));

  RCLCPP_INFO(get_logger(), "GoalDecisionNode started (state=WAIT_GAME_START).");
}

void GoalDecisionNode::loadConfig(const std::string & filepath)
{
  try {
    const YAML::Node root = YAML::LoadFile(filepath);

    if (root["goal_points"]) {
      for (const auto & node : root["goal_points"]) {
        const std::string name = node["name"].as<std::string>();
        GoalPoint gp;
        gp.frame_id = node["frame_id"].as<std::string>("map");
        gp.x = node["x"].as<double>(0.0);
        gp.y = node["y"].as<double>(0.0);
        gp.yaw = node["yaw"].as<double>(0.0);
        goal_points_[name] = gp;
        RCLCPP_INFO(get_logger(), "Loaded goal point '%s': frame=%s x=%.2f y=%.2f yaw=%.2f",
                    name.c_str(), gp.frame_id.c_str(), gp.x, gp.y, gp.yaw);
      }
    }

    if (root["decisions"]) {
      const auto & d = root["decisions"];
      game_start_progress_threshold_ = d["game_start_progress_threshold"].as<int>(4);
      hp_return_threshold_ = d["hp_return_threshold"].as<int>(30);
    }

    // ---- safety / robustness ----
    if (root["safety"]) {
      const auto & s = root["safety"];
      goal_timeout_sec_ = s["goal_timeout_sec"].as<double>(goal_timeout_sec_);
      max_retries_ = s["max_retries"].as<int>(max_retries_);
      retry_delay_sec_ = s["retry_delay_sec"].as<double>(retry_delay_sec_);
    }

    // ---- arbitration with manual goal sources (RViz / joystick) ----
    if (root["arbitration"]) {
      const auto & a = root["arbitration"];
      use_arbitration_ = a["enable"].as<bool>(use_arbitration_);
      arbitration_resume_delay_sec_ =
        a["resume_delay_sec"].as<double>(arbitration_resume_delay_sec_);
      arbitration_status_stale_sec_ =
        a["status_stale_sec"].as<double>(arbitration_status_stale_sec_);
    }

    // ---- localization readiness gate ----
    if (root["localization"]) {
      const auto & l = root["localization"];
      check_localization_ = l["check_ready"].as<bool>(check_localization_);
      localization_map_frame_ = l["map_frame"].as<std::string>(localization_map_frame_);
      localization_odom_frame_ = l["odom_frame"].as<std::string>(localization_odom_frame_);
    }

    if (goal_points_.find("center") == goal_points_.end() ||
        goal_points_.find("spawn") == goal_points_.end()) {
      RCLCPP_WARN(get_logger(),
                  "Config missing required points 'center' and/or 'spawn'!");
    }
    RCLCPP_INFO(get_logger(), "Config loaded: game_start_progress_threshold=%d, hp_return_threshold=%d%%",
                game_start_progress_threshold_, hp_return_threshold_);
    RCLCPP_INFO(get_logger(),
                "Safety: goal_timeout=%.1fs, max_retries=%d, retry_delay=%.1fs",
                goal_timeout_sec_, max_retries_, retry_delay_sec_);
    RCLCPP_INFO(get_logger(),
                "Arbitration: %s (resume_delay=%.1fs, status_stale=%.1fs); "
                "localization check: %s (%s -> %s)",
                use_arbitration_ ? "enabled" : "disabled",
                arbitration_resume_delay_sec_, arbitration_status_stale_sec_,
                check_localization_ ? "enabled" : "disabled",
                localization_map_frame_.c_str(), localization_odom_frame_.c_str());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Failed to load config '%s': %s",
                 filepath.c_str(), e.what());
  }
}

GoalDecisionNode::NavigateToPose::Goal
GoalDecisionNode::buildGoalMsg(const std::string & point_name)
{
  auto it = goal_points_.find(point_name);
  if (it == goal_points_.end()) {
    RCLCPP_ERROR(get_logger(), "Unknown goal point '%s'!", point_name.c_str());
    return NavigateToPose::Goal();
  }
  const GoalPoint & gp = it->second;

  geometry_msgs::msg::PoseStamped pose;
  pose.header.stamp = this->now();
  pose.header.frame_id = gp.frame_id;
  pose.pose.position.x = gp.x;
  pose.pose.position.y = gp.y;
  pose.pose.position.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, gp.yaw);
  pose.pose.orientation.x = q.x();
  pose.pose.orientation.y = q.y();
  pose.pose.orientation.z = q.z();
  pose.pose.orientation.w = q.w();

  NavigateToPose::Goal goal;
  goal.pose = pose;
  goal.behavior_tree = "";  // use default BT
  return goal;
}

void GoalDecisionNode::gameProgressCallback(
  const std_msgs::msg::UInt8::ConstSharedPtr msg)
{
  const int progress = static_cast<int>(msg->data);
  const bool started = (progress >= game_start_progress_threshold_);
  if (started != game_started_) {
    game_started_ = started;
    RCLCPP_INFO(get_logger(), "Game started = %d (game_progress=%d, threshold=%d)",
                game_started_, progress, game_start_progress_threshold_);
    onConditionsChanged();
  }
}

void GoalDecisionNode::currentHpCallback(
  const std_msgs::msg::Int32::ConstSharedPtr msg)
{
  current_hp_ = static_cast<int>(msg->data);
  recomputeHpLow();
}

void GoalDecisionNode::maximumHpCallback(
  const std_msgs::msg::Int32::ConstSharedPtr msg)
{
  maximum_hp_ = static_cast<int>(msg->data);
  recomputeHpLow();
}

void GoalDecisionNode::recomputeHpLow()
{
  const int cur = current_hp_;
  const int max = maximum_hp_;
  const bool low = (max > 0) && (cur <= static_cast<double>(max) * hp_return_threshold_ / 100.0);
  if (low != hp_low_) {
    hp_low_ = low;
    RCLCPP_INFO(get_logger(), "HP low = %d (current_hp=%d, max_hp=%d, threshold=%d%%)",
                hp_low_, cur, max, hp_return_threshold_);
    onConditionsChanged();
  }
}

void GoalDecisionNode::onConditionsChanged()
{
  if (!canSendGoals()) {
    return;  // action server / localization not ready, or a manual goal owns the action
  }

  // ---- 1) FSM transitions ----
  // Iteratively apply transitions until no more action is taken.
  for (int i = 0; i < 4; ++i) {
    bool acted = false;
    switch (state_) {
      case State::WAIT_GAME_START:
        if (game_started_) {
          retry_count_ = 0;
          goal_given_up_ = false;
          sendGoal("center");
          state_ = State::GOING_CENTER;
          acted = true;
        }
        break;
      case State::GOING_CENTER:
      case State::AT_CENTER:
        if (hp_low_) {
          retry_count_ = 0;
          goal_given_up_ = false;
          cancelCurrentGoal();
          sendGoal("spawn");
          state_ = State::GOING_SPAWN;
          acted = true;
        }
        break;
      case State::GOING_SPAWN:
      case State::AT_SPAWN:
        if (game_started_ && !hp_low_) {
          retry_count_ = 0;
          goal_given_up_ = false;
          sendGoal("center");
          state_ = State::GOING_CENTER;
          acted = true;
        }
        break;
    }
    if (!acted) {
      break;
    }
  }

  // ---- 2) recovery: a "GOING_x" state must always have a goal in flight ----
  // Covers goals that were preempted by a manual goal, or whose result was lost.
  // Once the retry budget for this state is used up we stop, so that this rule
  // can never turn into an endless resend loop.
  if (!goal_active_ && !retry_pending_ && !goal_given_up_) {
    if (state_ == State::GOING_CENTER) {
      RCLCPP_INFO(get_logger(), "No goal in flight while GOING_CENTER; re-sending 'center'.");
      sendGoal("center");
    } else if (state_ == State::GOING_SPAWN) {
      RCLCPP_INFO(get_logger(), "No goal in flight while GOING_SPAWN; re-sending 'spawn'.");
      sendGoal("spawn");
    }
  }
}

void GoalDecisionNode::sendGoal(const std::string & point_name)
{
  if (!action_client_->action_server_is_ready()) {
    return;
  }
  // Avoid re-sending the same target that is already active.
  if (goal_active_ && current_goal_name_ == point_name) {
    return;
  }

  const auto goal = buildGoalMsg(point_name);
  current_goal_name_ = point_name;
  goal_active_ = true;
  cancel_by_timeout_ = false;
  goal_start_time_valid_ = false;  // starts counting once the server accepts the goal
  retry_pending_ = false;

  RCLCPP_INFO(get_logger(), "Sending goal '%s' (frame=%s).",
              point_name.c_str(), goal.pose.header.frame_id.c_str());

  const std::string captured = point_name;
  action_client_->async_send_goal(
    goal,
    [this, captured](GoalHandle::SharedPtr goal_handle) {
      this->goalResponseCallback(goal_handle, captured);
    },
    [this, captured](GoalHandle::SharedPtr,
                     const std::shared_ptr<const NavigateToPose::Feedback> /*feedback*/) {
      (void)this;
      (void)captured;
      // feedback (e.g. distance_remaining) could be logged here if desired
    });
}

void GoalDecisionNode::cancelCurrentGoal()
{
  if (current_goal_handle_) {
    RCLCPP_INFO(get_logger(), "Canceling current goal '%s'.", current_goal_name_.c_str());
    action_client_->async_cancel_goal(current_goal_handle_);
  }
}

void GoalDecisionNode::goalResponseCallback(
  GoalHandle::SharedPtr goal_handle, const std::string & point_name)
{
  if (!goal_handle) {
    RCLCPP_ERROR(get_logger(), "Goal '%s' was rejected by server.", point_name.c_str());
    goal_active_ = false;
    goal_start_time_valid_ = false;
    scheduleRetry(point_name);
    return;
  }
  current_goal_handle_ = goal_handle;
  // Remember which goal is ours, so the arbitration callback can tell our goal
  // apart from goals sent by RViz / the joystick on the very same action.
  own_goal_uuid_ = goal_handle->get_goal_id();
  has_own_goal_uuid_ = true;
  goal_start_time_ = this->now();
  goal_start_time_valid_ = true;
  RCLCPP_INFO(get_logger(), "Goal '%s' accepted by server.", point_name.c_str());

  const std::string captured = point_name;
  auto result_future = goal_handle->get_result_async(
    [this, captured](const GoalHandle::WrappedResult & result) {
      this->resultCallback(result, captured);
    });
  (void)result_future;
}

void GoalDecisionNode::resultCallback(
  const GoalHandle::WrappedResult & result, const std::string & point_name)
{
  // Ignore stale results from a canceled/superseded goal.
  if (point_name != current_goal_name_) {
    return;
  }
  goal_active_ = false;
  goal_start_time_valid_ = false;
  current_goal_handle_.reset();

  const bool succeeded = (result.code == rclcpp_action::ResultCode::SUCCEEDED);
  const bool timed_out = cancel_by_timeout_;
  cancel_by_timeout_ = false;

  RCLCPP_INFO(get_logger(), "Goal '%s' finished (code=%d, success=%d, timed_out=%d).",
              point_name.c_str(), static_cast<int>(result.code), succeeded, timed_out);

  if (succeeded) {
    retry_count_ = 0;
    goal_given_up_ = false;
    if (point_name == "center") {
      state_ = State::AT_CENTER;
    } else if (point_name == "spawn") {
      state_ = State::AT_SPAWN;
    }
    // Re-evaluate: e.g. arrival at spawn + HP recovered + game on => back to center (loop).
    onConditionsChanged();
    return;
  }

  if (result.code == rclcpp_action::ResultCode::CANCELED) {
    if (timed_out) {
      // Canceled by our own timeout watchdog => count it as a failure.
      scheduleRetry(point_name);
    } else {
      // Canceled by us (preemption) or by another client (e.g. the joystick's
      // "cancel all goals"): stand by, the recovery rule resumes later.
      RCLCPP_WARN(get_logger(), "Goal '%s' was canceled externally; not retrying.",
                  point_name.c_str());
    }
    onConditionsChanged();
    return;
  }

  // ABORTED / UNKNOWN => navigation failed.
  if (external_goal_active_) {
    RCLCPP_WARN(get_logger(), "Goal '%s' failed while a manual goal is active; retry deferred.",
                point_name.c_str());
  } else {
    scheduleRetry(point_name);
  }
  onConditionsChanged();
}

bool GoalDecisionNode::canSendGoals() const
{
  if (!server_ready_) {
    return false;  // action server not up yet
  }
  if (check_localization_ && !localization_ready_) {
    return false;  // map -> odom missing => goals would be dropped/aborted by Nav2
  }
  if (use_arbitration_ && external_goal_active_) {
    return false;  // a manual goal (RViz / joystick) owns the action right now
  }
  return true;
}

void GoalDecisionNode::tfCallback(const tf2_msgs::msg::TFMessage::ConstSharedPtr msg)
{
  if (localization_ready_) {
    return;
  }
  for (const auto & transform : msg->transforms) {
    if (transform.header.frame_id == localization_map_frame_ &&
        transform.child_frame_id == localization_odom_frame_)
    {
      localization_ready_ = true;
      RCLCPP_INFO(get_logger(), "Localization is ready ('%s' -> '%s' seen on tf).",
                  localization_map_frame_.c_str(), localization_odom_frame_.c_str());
      onConditionsChanged();
      return;
    }
  }
}

void GoalDecisionNode::goalStatusCallback(
  const action_msgs::msg::GoalStatusArray::ConstSharedPtr msg)
{
  last_status_msg_time_ = this->now();

  bool external_active = false;
  for (const auto & status : msg->status_list) {
    const bool active =
      (status.status == action_msgs::msg::GoalStatus::STATUS_ACCEPTED) ||
      (status.status == action_msgs::msg::GoalStatus::STATUS_EXECUTING);
    if (!active) {
      continue;
    }
    if (has_own_goal_uuid_ && status.goal_info.goal_id.uuid == own_goal_uuid_) {
      continue;  // that one is ours
    }
    external_active = true;
    break;
  }

  if (external_active) {
    last_external_seen_ = this->now();
    if (!external_goal_active_) {
      external_goal_active_ = true;
      RCLCPP_INFO(get_logger(),
                  "A manual/external goal (RViz or joystick) became active; standing by.");
    }
  }
  // Clearing is done by the watchdog (needs a quiet period, see below), because
  // the joystick in auto_control mode restreams goals and leaves short gaps.
}

void GoalDecisionNode::scheduleRetry(const std::string & point_name)
{
  if (max_retries_ <= 0 || retry_count_ >= max_retries_) {
    goal_given_up_ = true;
    RCLCPP_ERROR(get_logger(),
                 "Goal '%s' failed and the retry budget (%d) is exhausted; giving up "
                 "until the conditions change again.", point_name.c_str(), max_retries_);
    return;
  }
  ++retry_count_;
  retry_pending_ = true;
  retry_point_ = point_name;
  retry_at_ = this->now() + rclcpp::Duration::from_seconds(retry_delay_sec_);
  RCLCPP_WARN(get_logger(), "Goal '%s' failed; retry %d/%d scheduled in %.1f s.",
              point_name.c_str(), retry_count_, max_retries_, retry_delay_sec_);
}

void GoalDecisionNode::watchdogTimerCallback()
{
  // ---- 1) action server readiness ----
  if (!server_ready_ && action_client_->action_server_is_ready()) {
    server_ready_ = true;
    RCLCPP_INFO(get_logger(), "navigate_to_pose action server is ready.");
    onConditionsChanged();
  }
  if (!server_ready_) {
    return;
  }

  const rclcpp::Time now = this->now();

  // ---- 2) hand the action back to the decision once manual goals go quiet ----
  if (external_goal_active_) {
    const double quiet = (now - last_external_seen_).seconds();
    const double stale = (now - last_status_msg_time_).seconds();
    if (quiet >= arbitration_resume_delay_sec_ || stale > arbitration_status_stale_sec_) {
      external_goal_active_ = false;
      RCLCPP_INFO(get_logger(),
                  "Manual/external goal released (quiet=%.1fs, no_status=%.1fs); resuming decision.",
                  quiet, stale);
      onConditionsChanged();
    }
  }

  // ---- 3) goal timeout: cancel and let the result callback retry ----
  if (goal_active_ && goal_start_time_valid_ &&
      (now - goal_start_time_).seconds() > goal_timeout_sec_)
  {
    if (!cancel_by_timeout_) {
      RCLCPP_WARN(get_logger(), "Goal '%s' exceeded the %.1f s timeout; canceling.",
                  current_goal_name_.c_str(), goal_timeout_sec_);
      cancel_by_timeout_ = true;
      timeout_cancel_issued_ = now;
      cancelCurrentGoal();
    } else if ((now - timeout_cancel_issued_).seconds() > kTimeoutCancelGraceSec) {
      // No CANCELED result came back => write the goal off locally and retry.
      RCLCPP_ERROR(get_logger(),
                   "No result %.1f s after canceling timed-out goal '%s'; writing it off.",
                   kTimeoutCancelGraceSec, current_goal_name_.c_str());
      const std::string failed_point = current_goal_name_;
      goal_active_ = false;
      goal_start_time_valid_ = false;
      cancel_by_timeout_ = false;
      current_goal_handle_.reset();
      scheduleRetry(failed_point);
      onConditionsChanged();
    }
  }

  // ---- 4) deferred retry ----
  if (retry_pending_ && (now - retry_at_).seconds() >= 0.0) {
    retry_pending_ = false;
    RCLCPP_WARN(get_logger(), "Retrying goal '%s' (%d/%d).",
                retry_point_.c_str(), retry_count_, max_retries_);
    sendGoal(retry_point_);
    if (!goal_active_) {
      onConditionsChanged();  // could not start a goal => re-evaluate
    }
  }
}

}  // namespace pb_nav_goal_decision

RCLCPP_COMPONENTS_REGISTER_NODE(pb_nav_goal_decision::GoalDecisionNode)
