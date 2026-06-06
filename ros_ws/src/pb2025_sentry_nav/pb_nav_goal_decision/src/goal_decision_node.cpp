#include "pb_nav_goal_decision/goal_decision_node.hpp"

#include <chrono>
#include <fstream>
#include <stdexcept>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "yaml-cpp/yaml.h"

namespace pb_nav_goal_decision
{

GoalDecisionNode::GoalDecisionNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("goal_decision_node", options),
  game_start_progress_threshold_(4),
  hp_return_threshold_(30),
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
  action_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

  // ---- wait for action server before issuing any goal ----
  server_check_timer_ = this->create_wall_timer(
    std::chrono::seconds(1),
    std::bind(&GoalDecisionNode::serverCheckTimerCallback, this));

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

    if (goal_points_.find("center") == goal_points_.end() ||
        goal_points_.find("spawn") == goal_points_.end()) {
      RCLCPP_WARN(get_logger(),
                  "Config missing required points 'center' and/or 'spawn'!");
    }
    RCLCPP_INFO(get_logger(), "Config loaded: game_start_progress_threshold=%d, hp_return_threshold=%d%%",
                game_start_progress_threshold_, hp_return_threshold_);
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
  if (!server_ready_) {
    return;  // cannot issue goals until the action server is up
  }

  // Iteratively apply transitions until no more action is taken.
  for (int i = 0; i < 4; ++i) {
    bool acted = false;
    switch (state_) {
      case State::WAIT_GAME_START:
        if (game_started_) {
          sendGoal("center");
          state_ = State::GOING_CENTER;
          acted = true;
        }
        break;
      case State::GOING_CENTER:
      case State::AT_CENTER:
        if (hp_low_) {
          cancelCurrentGoal();
          sendGoal("spawn");
          state_ = State::GOING_SPAWN;
          acted = true;
        }
        break;
      case State::GOING_SPAWN:
      case State::AT_SPAWN:
        if (game_started_ && !hp_low_) {
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
    return;
  }
  current_goal_handle_ = goal_handle;

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

  const bool succeeded = (result.code == rclcpp_action::ResultCode::SUCCEEDED);
  RCLCPP_INFO(get_logger(), "Goal '%s' finished (code=%d, success=%d).",
              point_name.c_str(), static_cast<int>(result.code), succeeded);

  if (point_name == "center") {
    state_ = State::AT_CENTER;
  } else if (point_name == "spawn") {
    state_ = State::AT_SPAWN;
  }

  // Re-evaluate: e.g. arrival at spawn + HP recovered + game on => go back to center (loop).
  onConditionsChanged();
}

void GoalDecisionNode::serverCheckTimerCallback()
{
  if (!server_ready_ && action_client_->action_server_is_ready()) {
    server_ready_ = true;
    RCLCPP_INFO(get_logger(), "navigate_to_pose action server is ready.");
    onConditionsChanged();
  }
}

}  // namespace pb_nav_goal_decision

RCLCPP_COMPONENTS_REGISTER_NODE(pb_nav_goal_decision::GoalDecisionNode)
