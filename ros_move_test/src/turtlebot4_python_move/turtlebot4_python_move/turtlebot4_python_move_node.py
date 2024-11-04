from irobot_create_msgs.msg import InterfaceButtons, LightringLeds
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import math
import time


class TurtleBot4FirstNode(Node):

    def __init__(self):
        super().__init__('Move_node')
        self.get_logger().info('Begin')
        # Subscribe to the /interface_buttons topic
        self.interface_buttons_subscriber = self.create_subscription(
            InterfaceButtons,
            '/interface_buttons',
            self.interface_buttons_callback,
            qos_profile_sensor_data)

        # Create a publisher for the /cmd_vel topic
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

    # Interface buttons subscription callback
    def interface_buttons_callback(self, create3_buttons_msg: InterfaceButtons):
        #对应2号按键（在机器人不处于充电状态时，1号按键默认功能为返回充电桩）
        if create3_buttons_msg.button_2.is_pressed:
            self.get_logger().info('Button 2 Pressed!')
            self.button_2_function()

    
    def button_2_function(self):
        #可以自行选择2号按键的回调函数
        # self.forward_callback(distance=0.5, speed=0.1)
        while True:
            self.rotate_callback(angle=90, speed=30)
        # self.forward_callback(distance=0.5, speed=0.1)
        # self.rotate_callback(angle=90, speed=30)
        # self.forward_callback(distance=0.5, speed=0.1)
        # self.rotate_callback(angle=90, speed=30)
        # self.forward_callback(distance=0.5, speed=0.1)
        # self.rotate_callback(angle=90, speed=30)
    
    
    def rotate_callback(self,angle,speed):
        #angle为旋转角度，单位度；speed为旋转角速度，单位度/s，正负对应不同旋转方向
        # 将角度转换为弧度
        radians = math.radians(angle)
        speed = math.radians(speed)
        # 计算旋转所需时间（秒）
        time_to_rotate = radians / abs(speed)
        # 计算旋转所需的消息数量
        num_messages = int(time_to_rotate / 0.1)
        for i in range(num_messages):
            twist_msg = Twist()
            twist_msg.linear.x = 0.0  # 线速度为0，保持位置
            twist_msg.angular.z = speed  # 角速度
            self.publisher_.publish(twist_msg)
            self.get_logger().info('Rotating...')
            time.sleep(0.1)  # 等待0.1秒

        # 发布一个停止命令
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        self.publisher_.publish(twist_msg)
        self.get_logger().info('Rotation complete.')
    
    def forward_callback(self,distance,speed):
        #distance为直行的距离，单位为m；speed单位m/s
        time_to_rotate = distance / abs(speed)
        num_messages = int(time_to_rotate / 0.1)
        for i in range(num_messages):
            twist_msg = Twist()
            twist_msg.linear.x = speed
            twist_msg.angular.z = 0.0
            self.publisher_.publish(twist_msg)
            self.get_logger().info('Forwading...')
            time.sleep(0.1)  # 等待0.1秒
        # 发布一个停止命令
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        self.publisher_.publish(twist_msg)
        self.get_logger().info('Forward complete.')

def main(args=None):
    rclpy.init(args=args)
    node = TurtleBot4FirstNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
