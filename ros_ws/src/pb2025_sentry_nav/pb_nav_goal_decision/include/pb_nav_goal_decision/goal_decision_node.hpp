#ifndef PB_NAV_GOAL_DECISION__GOAL_DECISION_NODE_HPP_
#define PB_NAV_GOAL_DECISION__GOAL_DECISION_NODE_HPP_

#include <memory>
#include <string>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

// 输入用标准通信包（std_msgs），不再依赖自定义 pb_rm_interfaces
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/int32.hpp"

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

  // ---- server readiness ----
  void serverCheckTimerCallback();

  // subscriptions
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr game_progress_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr current_hp_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr maximum_hp_sub_;

  // action client to Nav2 bt_navigator
  rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
  GoalHandle::SharedPtr current_goal_handle_;

  // readiness timer
  rclcpp::TimerBase::SharedPtr server_check_timer_;

  // ---- config data ----
  std::unordered_map<std::string, GoalPoint> goal_points_;
  int game_start_progress_threshold_;  // game_progress >= this => game started
  int hp_return_threshold_;           // percent of maximum_hp

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
};

}  // namespace pb_nav_goal_decision

#endif  // PB_NAV_GOAL_DECISION__GOAL_DECISION_NODE_HPP_
