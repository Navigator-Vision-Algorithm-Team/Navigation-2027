# Standalone launcher for the goal decision node (for debugging).
#
#   ros2 launch pb_nav_goal_decision goal_decision_launch.py
#
# IMPORTANT: `namespace` is NOT optional. The node talks to Nav2 with relative
# names, so it must run in the SAME namespace as the navigation stack:
#   * action            -> <ns>/navigate_to_pose          (and its _action/status)
#   * referee inputs    -> <ns>/referee/*
#   * localization gate -> <ns>/tf and <ns>/tf_static
# With the default nav2 namespace that means uncommenting the line below;
# otherwise the node would look for /tf, /navigate_to_pose, ... at the root and
# simply never receive anything (it would sit idle and log nothing useful).

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
            namespace='red_standard_robot1',
        ),
    ])
