import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from nav_msgs.msg import Odometry
import tf2_ros
import tf2_geometry_msgs

class NavigateToPoseClient(Node):
    def __init__(self):
        super().__init__('navigate_to_pose_client')

        # 初始化 ActionClient
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # 初始化订阅器，用于订阅位姿信息
        self._odom_subscriber = self.create_subscription(
            Odometry,
            '/odom',  # 订阅 Odom topic
            self.odom_callback,
            10
        )
        
        # 初始化 TF监听器，用于获取transform信息
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def send_goal(self, goal_pose):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose

        self.get_logger().info('Sending goal to navigate to pose')

        # 发送目标点
        self._action_client.send_goal_async(goal_msg)

    def odom_callback(self, msg):
        # 获取Odom坐标系下的机器人位姿
        self.get_logger().info(f'Odometry - Position: x={msg.pose.pose.position.x}, y={msg.pose.pose.position.y}')

    def get_robot_pose_in_map(self):
        try:
            # 获取从 'map' 坐标系到 'base_link'（假设是机器人的底盘）的坐标变换
            transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            
            # 通过 transform 获取机器人位姿
            robot_pose = transform.transform.translation
            self.get_logger().info(f'Robot Pose in Map - x={robot_pose.x}, y={robot_pose.y}')
        except tf2_ros.LookupException as e:
            self.get_logger().warn(f'Could not get transform: {str(e)}')

def main(args=None):
    rclpy.init(args=args)

    navigate_to_pose_client = NavigateToPoseClient()

    # 等待 Action Server 启动
    if not navigate_to_pose_client._action_client.wait_for_server(timeout_sec=5.0):
        navigate_to_pose_client.get_logger().error('Action server not available')
        return

    # 创建目标位置
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.pose.position.x = 2.0  # 目标点的X坐标
    goal_pose.pose.position.y = 3.0  # 目标点的Y坐标
    goal_pose.pose.orientation.w = 1.0  # 目标点的四元数旋转

    # 发送目标点
    navigate_to_pose_client.send_goal(goal_pose.pose)

    # 获取机器人当前位置
    navigate_to_pose_client.get_robot_pose_in_map()

    # 让程序运行，直到手动退出
    rclpy.spin(navigate_to_pose_client)

    rclpy.shutdown()

if __name__ == '__main__':
    main()

