import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO


class YoloRealSenseNode(Node):
    def __init__(self):
        super().__init__('yolo_realsense_node')

        # YOLOモデルの読み込み
        self.model = YOLO('/home/yanagi/models/yolov8n.pt')  # モデルファイルのパスを指定
        self.bridge = CvBridge()

        # RealSenseカメラの設定
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)
        #self.config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 30)
        self.pipeline.start(self.config)

        # ROS2サブスクリプションの設定
        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',  # RealSenseの画像トピック
            self.image_callback,
            10)
        self.subscription  # prevent unus    

        self.pub=self.create_publisher(Twist,'/cmd_vel',10)               

    def image_callback(self, msg):
        # 画像メッセージをOpenCV形式に変換
        color_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        
        # YOLOで推論
        results = self.model(color_image,classes=[0])
        
        # 推論結果を描画
        annotated_img = results[0].plot()

        for i, box in enumerate(results[0].boxes.xyxy):
            class_id = int(results[0].boxes.cls[i])
            class_name = self.model.names[class_id]
            x1, y1, x2, y2 = map(int, box[:4])
            x_center = (x1 + x2) // 2
            y_center = (y1 + y2) // 2

            if class_name=='person':



                #depth_frame = self.pipeline.wait_for_frames().get_depth_frame()
                #distance = depth_frame.get_distance(x_center, y_center)
                twist_msg=Twist()
                twist_msg.linear.x = 0.5  # 前進の速度
                twist_msg.angular.z = 0.0  # 旋回の速度


                # cmd_velトピックにパブリッシュ
                self.pub.publish(twist_msg)
                self.get_logger().info(f'人間ミッケ！')
            
                



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
