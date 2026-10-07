#ifndef PB_NAV_GOAL_DECISION__GOAL_DECISION_NODE_HPP_
#define PB_NAV_GOAL_DECISION__GOAL_DECISION_NODE_HPP_

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "action_msgs/msg/goal_status_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

// 输入用标准通信包（std_msgs），不再依赖自定义 pb_rm_interfaces
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/int32.hpp"

// 仅用于"定位是否就绪"的检测（监听 tf 上是否已出现 map -> odom 变换）
#include "tf2_msgs/msg/tf_message.hpp"

namespace pb_nav_goal_decision
{

// A single named point loaded from the YAML config.
struct GoalPoint
{
  std::string frame_id;
  double x;
  double y;
  double yaw;  // radians
};

class GoalDecisionNode : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  explicit GoalDecisionNode(const rclcpp::NodeOptions & options);

private:
  // ---- config ----
  void loadConfig(const std::string & filepath);
  NavigateToPose::Goal buildGoalMsg(const std::string & point_name);

  // ---- subscriptions (std_msgs, relayed by the forwarding/serial node) ----
  void gameProgressCallback(const std_msgs::msg::UInt8::ConstSharedPtr msg);
  void currentHpCallback(const std_msgs::msg::Int32::ConstSharedPtr msg);
  void maximumHpCallback(const std_msgs::msg::Int32::ConstSharedPtr msg);
  void recomputeHpLow();

  // ---- decision core ----
  void onConditionsChanged();   // (re)evaluate FSM based on current conditions
  void sendGoal(const std::string & point_name);
  void cancelCurrentGoal();

  // ---- action client callbacks ----
  void goalResponseCallback(GoalHandle::SharedPtr goal_handle, const std::string & point_name);
  void resultCallback(const GoalHandle::WrappedResult & result, const std::string & point_name);

  // ---- arbitration: goals sent by other clients (RViz / joystick) ----
  void goalStatusCallback(const action_msgs::msg::GoalStatusArray::ConstSharedPtr msg);

  // ---- readiness & watchdog (server / localization / timeout / retry) ----
  void watchdogTimerCallback();
  bool canSendGoals() const;
  void scheduleRetry(const std::string & point_name);

  // ---- localization readiness: look for `map -> odom` on the tf topics ----
  void tfCallback(const tf2_msgs::msg::TFMessage::ConstSharedPtr msg);

  // subscriptions
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr game_progress_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr current_hp_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr maximum_hp_sub_;
  // <action>/_action/status -- the only way to see goals sent by OTHER clients
  rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr goal_status_sub_;

  // localization readiness: relative names => resolve to <namespace>/tf(_static),
  // i.e. exactly where the relocalization node broadcasts `map -> odom`.
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_static_sub_;

  // action client to Nav2 bt_navigator
  rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
  GoalHandle::SharedPtr current_goal_handle_;

  // watchdog timer (2 Hz): server / localization / timeout / retry / resume
  rclcpp::TimerBase::SharedPtr watchdog_timer_;

  // ---- config data ----
  std::unordered_map<std::string, GoalPoint> goal_points_;
  int game_start_progress_threshold_;  // game_progress >= this => game started
  int hp_return_threshold_;           // percent of maximum_hp

  // ---- config: safety / robustness ----
  double goal_timeout_sec_;           // a single goal longer than this is canceled
  int max_retries_;                   // retries allowed for one target point
  double retry_delay_sec_;            // wait before a retry

  // ---- config: arbitration with manual goal sources ----
  bool use_arbitration_;              // stand by while another client owns the action
  double arbitration_resume_delay_sec_;  // quiet time before resuming after a manual goal
  double arbitration_status_stale_sec_;  // no status updates => assume action is free

  // ---- config: localization readiness ----
  bool check_localization_;           // gate goals on map->odom being available
  std::string localization_map_frame_;
  std::string localization_odom_frame_;

  // ---- latest referee values (from std_msgs topics) ----
  int current_hp_{0};
  int maximum_hp_{0};

  // ---- runtime state ----
  enum class State
  {
    WAIT_GAME_START,
    GOING_CENTER,
    AT_CENTER,
    GOING_SPAWN,
    AT_SPAWN
  };
  State state_;
  bool game_started_;
  bool hp_low_;
  std::string current_goal_name_;
  bool goal_active_;
  bool server_ready_;

  // ---- runtime: readiness / arbitration ----
  bool localization_ready_{false};
  bool external_goal_active_{false};      // a manual/external goal owns the action
  rclcpp::Time last_external_seen_;       // last time an external goal was seen active
  rclcpp::Time last_status_msg_time_;     // last time the action status was received

  // ---- runtime: own goal identity (to tell our goals from other clients') ----
  std::array<uint8_t, 16> own_goal_uuid_{};
  bool has_own_goal_uuid_{false};

  // ---- runtime: timeout & retry ----
  rclcpp::Time goal_start_time_;
  bool goal_start_time_valid_{false};
  bool cancel_by_timeout_{false};
  rclcpp::Time timeout_cancel_issued_;  // when the timeout cancel was requested
  int retry_count_{0};
  bool retry_pending_{false};
  bool goal_given_up_{false};  // retry budget exhausted for the current state
  std::string retry_point_;
  rclcpp::Time retry_at_;
};

}  // namespace pb_nav_goal_decision

#endif  // PB_NAV_GOAL_DECISION__GOAL_DECISION_NODE_HPP_
