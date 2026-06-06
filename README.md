# Navigation-2027

2027 导航代码。

这套导航需要配合自瞄端的串口通信使用。使用前请先保证自瞄程序已经正常运行，串口设备已连接，并且导航端能够通过串口节点收到/发送机器人状态与控制相关数据。

## 目录说明

- `ros_ws/`: ROS2 工作区，导航、建图、仿真和相关依赖包都在这里。
- `nav.py`, `serial_node.py`, `yaw.py`: 顶层辅助脚本。
- `nav_run.sh`, `to_home.sh`, `to_center.sh`: 常用启动或目标点脚本。
- `ros_ws/src/pb2025_sentry_nav/pb2025_nav_bringup/`: 主要导航启动文件、参数、地图和点云文件所在位置。

## 使用前准备

进入 ROS 工作区：

```bash
cd ros_ws
```

加载 ROS2 工作区环境：

```bash
source install/setup.bash
```

如果当前终端没有加载 ROS2 Humble，也需要先加载系统 ROS 环境：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

## 启动串口

导航启动前需要先打开串口节点。串口依赖自瞄端配合，确认自瞄程序已经启动后，在一个单独终端中执行：

```bash
cd ros_ws
source install/setup.bash
ros2 launch rm_serial_driver serial_driver.launch.py
```

串口节点正常后，再启动建图或导航模式。

## 建图模式

建图时开启 `slam:=True`：

```bash
cd ros_ws
source install/setup.bash
ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py \
  slam:=True \
  use_robot_state_pub:=True
```

## 导航模式

导航时关闭 SLAM，并指定地图名：

```bash
cd ros_ws
source install/setup.bash
ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py \
  world:=final2 \
  slam:=False \
  use_robot_state_pub:=True
```

其中 `world:=final2` 里的 `final2` 是地图名字，可以根据实际使用的地图修改。比如地图文件名换成 `dona` 时，就把参数改成：

```bash
world:=dona
```

## 注意事项

- 启动顺序建议为：先启动自瞄程序，再启动串口节点，最后启动建图或导航。
- `build/`, `install/`, `log/` 不建议提交到 GitHub，换机器后应重新编译生成。
- 大部分 `.pcd` 点云地图没有上传到仓库，当前只保留了 `dona.pcd`。
