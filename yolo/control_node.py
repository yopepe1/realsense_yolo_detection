import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

class TwistNode(Node):
    def __init__(self):
        super().__init__('twist_node')

        # ROS2のパブリッシャー
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # サブスクリプション設定
        self.create_subscription(Bool, 'green_detected', self.green_callback, 10)
        self.create_subscription(Bool, 'red_detected', self.red_callback, 10)

        self.twist_msg = Twist()

    def green_callback(self, msg):
        if msg.data:  # GREENが検知された場合
            self.twist_msg.linear.x = 0.0  # 止まる
        self.publish_twist()

    def red_callback(self, msg):
        if msg.data:  # REDが検知された場合
            self.twist_msg.linear.x = 1.0  # 1m/sで進む
        self.publish_twist()

    def publish_twist(self):
        self.pub.publish(self.twist_msg)
        self.get_logger().info(f'速度: {self.twist_msg.linear.x} m/s')


def main(args=None):
    rclpy.init(args=args)
    twist_node = TwistNode()
    rclpy.spin(twist_node)
    twist_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
