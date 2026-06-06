# Navigation-2027

2027 导航代码。

使用前需要搭配自瞄的串口程序/串口节点。先打开串口通信，再启动建图或导航相关 launch。

## 启动流程

进入 ROS 工作区：

```bash
cd ros_ws
```

加载环境：

```bash
source install/setup.bash
```

启动串口：

```bash
ros2 launch rm_serial_driver serial_driver.launch.py
```

建图模式：

```bash
ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py \
  slam:=True \
  use_robot_state_pub:=True
```

导航模式：

```bash
source install/setup.bash
```
