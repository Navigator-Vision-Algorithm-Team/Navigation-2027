#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import serial
import struct
import time
import crcmod
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist

# 初始化 CRC16-MODBUS 函数
crc16 = crcmod.predefined.mkCrcFun('modbus')

def crc8_maxim(data: bytes):
    """计算CRC8-MAXIM校验"""
    crc = 0x00
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = (crc << 1) ^ 0x31
            else:
                crc <<= 1
            crc &= 0xFF
    return crc

class SerialNode(Node):
    def __init__(self):
        super().__init__('serial_node')
        self.get_logger().info('串口通信节点启动...')

        # 串口参数
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 921600)
        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baud_rate').value

        # 串口初始化
        try:
            self.serial_port = serial.Serial(port=port, baudrate=baud, timeout=0.1)
            self.get_logger().info(f'串口已打开: {port} @ {baud}')
        except serial.SerialException as e:
            self.get_logger().error(f'串口打开失败: {str(e)}')
            raise e

        # 发布器: 云台姿态
        self.imu_pub = self.create_publisher(Float32MultiArray, '/serial/gimbal_joint_state', 10)

        # 订阅器: cmd_vel
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/red_standard_robot1/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.seq = 0  # 包序号

        # 定时器 100Hz
        self.timer = self.create_timer(0.01, self.timer_callback)

    def timer_callback(self):
        """周期任务：接收云台数据"""
        try:
            if self.serial_port.in_waiting >= 21:
                data = self.serial_port.read(21)

                if data[0] != 0xA5:
                    # self.get_logger().warning("帧头错误，丢弃数据")
                    return

                unpacked = struct.unpack('<BHBHBfffH', data)
                # unpacked: (header, size, id, crc8, cmd_id, pitch, roll, yaw, crc16)

                pitch = unpacked[5]
                roll = unpacked[6]
                yaw = unpacked[7]

                # 发布姿态数据
                msg = Float32MultiArray()
                msg.data = [pitch, yaw]
                self.imu_pub.publish(msg)

                self.get_logger().info(f"[接收] Pitch: {pitch:.2f}, Roll: {roll:.2f}, Yaw: {yaw:.2f}")

        except Exception as e:
            self.get_logger().error(f"接收错误: {str(e)}")

    def cmd_vel_callback(self, msg):
        """收到cmd_vel，发送控制指令"""
        self.send_cmd_vel(msg.linear.x, msg.linear.y, msg.angular.z)

    def send_cmd_vel(self, vx, vy, wz):
        """构造并发送控制指令 (cmd_id = 0x0405)"""
        try:
            header = 0xA5
            data_len = 12  # 3个float
            cmd_id = 0x0405

            self.seq = (self.seq + 1) % 256  # 包序号循环

            # 构建帧头部分
            head_bytes = struct.pack('<BHB', header, data_len, self.seq)
            crc8_val = crc8_maxim(head_bytes)

            # 指令ID + 数据体
            cmd_and_data = struct.pack('<Hfff', cmd_id, vx, vy, wz)

            # 拼接完整数据
            full_data = head_bytes + struct.pack('B', crc8_val) + cmd_and_data

            # 添加CRC16
            crc16_val = crc16(full_data)
            full_data += struct.pack('<H', crc16_val)

            self.serial_port.write(full_data)
            self.get_logger().info(f"[发送cmd_vel] vx:{vx:.2f}, vy:{vy:.2f}, wz:{wz:.2f}")

        except Exception as e:
            self.get_logger().error(f"发送cmd_vel失败: {str(e)}")

    def __del__(self):
        if hasattr(self, 'serial_port') and self.serial_port.is_open:
            self.serial_port.close()

def main(args=None):
    rclpy.init(args=args)
    node = SerialNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
