import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class YoloNode(Node):
    def __init__(self):
        super().__init__('yolo_node')

        # YOLOモデルの読み込み
        self.model = YOLO('/home/yoheiyanagi/YOLO/ultralytics/runs/detect/train2/weights/best.pt')
        self.bridge = CvBridge()
        
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        

        # ROS2サブスクリプションの設定
        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            10)
        self.subscription  # prevent unused variable warning

        # GREENとREDの検知状態をパブリッシュするためのパブリッシャー
        self.green_pub = self.create_publisher(Bool, 'green_detected', 10)
        self.red_pub = self.create_publisher(Bool, 'red_detected', 10)

    def image_callback(self, msg):
        # 画像メッセージをOpenCV形式に変換
        color_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        # YOLOで推論
        results = self.model(color_image)

        green_detected = False
        red_detected = False

        # 結果を解析
        for i, box in enumerate(results[0].boxes.xyxy):
            class_id = int(results[0].boxes.cls[i])
            if class_id == 0:  # GREEN
                green_detected = True
            elif class_id == 1:  # RED
                red_detected = True

        # GREENとREDの状態をパブリッシュ
        self.green_pub.publish(Bool(data=green_detected))
        self.red_pub.publish(Bool(data=red_detected))

        self.get_logger().info(f'GREEN検知: {green_detected}, RED検知: {red_detected}')


def main(args=None):
    rclpy.init(args=args)
    yolo_node = YoloNode()
    rclpy.spin(yolo_node)
    yolo_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
