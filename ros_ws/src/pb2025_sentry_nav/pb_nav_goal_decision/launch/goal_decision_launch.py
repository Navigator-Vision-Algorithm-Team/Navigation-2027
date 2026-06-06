# Standalone launcher for the goal decision node.
#
# NOTE: This node is intentionally NOT wired into the navigation bringup chain
# (bringup_launch.py / navigation_launch.py) yet. Run it separately for testing:
#
#   ros2 launch pb_nav_goal_decision goal_decision_launch.py
#
# If the referee topics / navigate_to_pose action live under a namespace
# (e.g. red_standard_robot1), uncomment the `namespace` line so the node joins
# the same namespace and can see those topics.

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pb_nav_goal_decision',
            executable='goal_decision_node',
            name='goal_decision_node',
            output='screen',
            parameters=[{
                'config_file': '',  # empty => default share path config/goal_decision_params.yaml
            }],
            # namespace='red_standard_robot1',
        ),
    ])
