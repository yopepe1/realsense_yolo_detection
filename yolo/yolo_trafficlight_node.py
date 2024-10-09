import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist  # 変更：Twistメッセージをインポート
from cv_bridge import CvBridge
import cv2
import pyrealsense2 as rs
from ultralytics import YOLO

class YoloRealSenseNode(Node):
    def __init__(self):
        super().__init__('yolo_realsense_node')

        # YOLOモデルの読み込み
        self.model = YOLO('/home/yanagi/yolo learning data/best.pt')  # データセット用のモデルパスを指定
        self.bridge = CvBridge()

        # RealSenseカメラの設定
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)
        self.pipeline.start(self.config)

        # ROS2のパブリッシャー
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)  # 変更：Twistメッセージ用に変更

        # ROS2サブスクリプションの設定
        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',  # カメラの画像トピック
            self.image_callback,
            10)
        self.subscription  # prevent unused variable warning                   

    def image_callback(self, msg):
        # 画像メッセージをOpenCV形式に変換
        color_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        # YOLOで推論
        results = self.model(color_image)

        # 推論結果を描画
        annotated_img = results[0].plot()

        green_detected = False
        red_detected = False

        # 結果を解析
        for i, box in enumerate(results[0].boxes.xyxy):
            class_id = int(results[0].boxes.cls[i])
            if class_id == 0:  # GREEN
                green_detected = True
            elif class_id == 1:  # RED
                red_detected = True

        # 条件に応じてTwistメッセージを送信
        twist_msg = Twist()

        if green_detected:
            twist_msg.linear.x = 0.0  # GREENが検知された場合、止まる
        elif red_detected:
            twist_msg.linear.x = 1.0  # REDが検知された場合、1m/sで進む
        else:
            twist_msg.linear.x = 0.0  # どちらも検知されなかった場合、止まる

        # トピックにTwistメッセージをパブリッシュ
        self.pub.publish(twist_msg)
        self.get_logger().info(f'速度: {twist_msg.linear.x} m/s')

        # 結果を表示
        cv2.imshow('YOLO RealSense', annotated_img)
        cv2.waitKey(1)


def destroy_node(self):
        # パイプラインとウィンドウを終了
        self.pipeline.stop()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    yolo_realsense_node = YoloRealSenseNode()
    rclpy.spin(yolo_realsense_node)
    yolo_realsense_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
