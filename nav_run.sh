#!/bin/bash

sleep 60

# 增益点 
ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped "{header: {frame_id: 'map'}, pose: {position: {x:6.3, y:-4.3, z:0.0}, orientation: {x:0.0, y:0.0, z:0.0, w:1.0}}}"

sleep 90
# 回家
ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped "{header: {frame_id: 'map'}, pose: {position: {x:0.0, y:0.0, z:0.0}, orientation: {x:0.0, y:0.0, z:0.0, w:1.0}}}"
