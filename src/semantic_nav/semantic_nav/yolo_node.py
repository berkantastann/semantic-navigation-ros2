#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
from ament_index_python.packages import get_package_share_directory
from ultralytics import YOLO

class YoloNode(Node):
    def __init__(self):
        super().__init__('yolo_node')
        
        self.camera_sub = self.create_subscription(
            Image,
            '/camera/image',
            self.listener_callback,
            10)
        
        self.bridge = CvBridge()
        
        package_share_directory = get_package_share_directory('semantic_nav')
        model_path = os.path.join(package_share_directory, 'models', 'yolov8n.onnx')
        
        self.model = YOLO(model_path)
        self.get_logger().info(f"Model yükleniyor: {model_path}")
        
        try:
            self.model = YOLO(model_path, task='detect')
            self.get_logger().info("Model başarıyla yüklendi.")
        except Exception as e:
            self.get_logger().error(f"Model yüklenirken hata oluştu: {e}")
            
    def listener_callback(self, msg):
        try:
            current_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            results = self.model.predict(current_frame, conf=0.5,verbose=False)
            annotated_frame = results[0].plot()
            
            for r in results:
                for box in r.boxes:
                    cls_id = int(box.cls[0])
                    cls_name = self.model.names[cls_id]
                    self.get_logger().info(f"Nesne: {cls_name}")   
            
            cv2.namedWindow("Robot Gozu", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("Robot Gozu", 600, 400)
            cv2.imshow("Robot Gozu", annotated_frame)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f"Görüntü işleme hatası: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = YoloNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()         
        

    