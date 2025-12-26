#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.duration import Duration
from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
import message_filters
from ultralytics import YOLO
import os
import json
from ament_index_python.packages import get_package_share_directory
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs

class SemanticMapper(Node):
    def __init__(self):
        super().__init__('semantic_mapper')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        package_share_directory = get_package_share_directory('semantic_nav')
        model_path = os.path.join(package_share_directory, 'models', 'yolov8n.onnx')
        
        self.detected_objects = [] 
        self.db_path = os.path.expanduser("~/semantic_db.json")
        
        if os.path.exists(self.db_path):
            try:
                with open(self.db_path, 'r') as f:
                    self.detected_objects = json.load(f)
            except:
                pass

        try:
            self.model = YOLO(model_path, task='detect')
            self.get_logger().info("YOLO Hazır 🚀")
        except Exception as e:
            self.get_logger().error(f"Model hatası: {e}")

        self.rgb_sub = message_filters.Subscriber(
            self, Image, '/camera/image', qos_profile=qos_profile_sensor_data)
        
        self.depth_sub = message_filters.Subscriber(
            self, Image, '/camera/depth_image', qos_profile=qos_profile_sensor_data)
        
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub], queue_size=10, slop=0.3)
        self.ts.registerCallback(self.sync_callback)

        self.create_subscription(
            CameraInfo, '/camera/camera_info', self.info_callback, qos_profile_sensor_data)
            
        self.fx = None
        self.marker_pub = self.create_publisher(MarkerArray, '/semantic_markers', 10)
        self.br = CvBridge()
        
        cv2.namedWindow("Robot Gozu", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Robot Gozu", 640, 480)

    def info_callback(self, msg):
        if self.fx is None:
            self.fx = msg.k[0]
            self.cx = msg.k[2]
            self.fy = msg.k[4]
            self.cy = msg.k[5]

    def sync_callback(self, rgb_msg, depth_msg):
        if self.fx is None: return 

        try:
            cv_image = self.br.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
            
            try:
                cv_depth = self.br.imgmsg_to_cv2(depth_msg, desired_encoding='32FC1')
            except:
                cv_depth = self.br.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')

            results = self.model.predict(cv_image, conf=0.6, verbose=False)
            
            marker_array = MarkerArray()
            new_object_saved = False 

            for r in results:
                for box in r.boxes:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    u = int((x1 + x2) / 2)
                    v = int((y1 + y2) / 2)
                    
                    cls_id = int(box.cls[0])
                    cls_name = self.model.names[cls_id]

                    roi_size = 5
                    u_min = max(0, u - roi_size); u_max = min(cv_depth.shape[1], u + roi_size)
                    v_min = max(0, v - roi_size); v_max = min(cv_depth.shape[0], v + roi_size)
                    
                    roi = cv_depth[v_min:v_max, u_min:u_max]
    
                    if roi.size == 0: continue
                    
                    if np.all(np.isnan(roi)): continue
                    
                    depth = np.nanmedian(roi)

                    if np.isnan(depth) or np.isinf(depth) or depth < 0.3 or depth > 8.0:
                        continue

                    z_cam = float(depth)
                    x_cam = (u - self.cx) * z_cam / self.fx
                    y_cam = (v - self.cy) * z_cam / self.fy

                    try:
                        point_cam = PointStamped()
                        point_cam.header.frame_id = "camera_depth_optical_frame"
                        point_cam.header.stamp = depth_msg.header.stamp
                        point_cam.point.x = x_cam
                        point_cam.point.y = y_cam
                        point_cam.point.z = z_cam

                        # Transform 
                        if self.tf_buffer.can_transform("map", point_cam.header.frame_id, depth_msg.header.stamp, Duration(seconds=0.1)):
                            point_map = self.tf_buffer.transform(point_cam, "map")
                            
                            # Z-Ekseni Filtresi (Yerin dibi/tavan harici)
                            if -0.5 < point_map.point.z < 1.0:
                                # Kayıt
                                if self.check_and_save(cls_name, point_map.point.x, point_map.point.y):
                                    new_object_saved = True
                                
                                # Görselleştirme
                                self.add_marker(marker_array, point_map.point, cls_name)

                    except Exception:
                        pass 
            
            # Disk Yazma ve Marker Yayını
            if new_object_saved:
                self.flush_to_disk()
                self.marker_pub.publish(marker_array)
            elif len(marker_array.markers) > 0:
                self.marker_pub.publish(marker_array)

            # Görüntü
            annotated_frame = results[0].plot()
            cv2.imshow("Robot Gozu", annotated_frame)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f"Döngü hatası: {e}")

    def check_and_save(self, name, x, y):
        """RAM kontrolü"""
        for obj in self.detected_objects:
            if obj['name'] == name:
                dist = np.sqrt((obj['x'] - x)**2 + (obj['y'] - y)**2)
                if dist < 1.5: 
                    return False

        new_obj = {"name": name, "x": x, "y": y}
        self.detected_objects.append(new_obj)
        self.get_logger().warn(f"📍 TESPİT EDİLDİ: {name} (X:{x:.2f}, Y:{y:.2f})")
        return True

    def flush_to_disk(self):
        """Diske yazma"""
        try:
            with open(self.db_path, 'w') as f:
                json.dump(self.detected_objects, f, indent=4)
        except Exception as e:
            self.get_logger().error(f"Dosya yazma hatası: {e}")

    def add_marker(self, marker_array, point, text):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position = point
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.3; marker.scale.y = 0.3; marker.scale.z = 0.3
        marker.color.a = 0.9; marker.color.r = 1.0; marker.color.g = 0.0; marker.color.b = 0.0
        marker.id = abs(hash(text)) % 10000 
        marker.lifetime = Duration(seconds=1.0).to_msg() 
        marker_array.markers.append(marker)

        text_marker = Marker()
        text_marker.header.frame_id = "map"
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        text_marker.pose.position = point
        text_marker.pose.position.z += 0.4
        text_marker.scale.z = 0.3
        text_marker.color.a = 1.0; text_marker.color.r = 1.0; text_marker.color.g = 1.0; text_marker.color.b = 1.0
        text_marker.text = text
        text_marker.id = (abs(hash(text)) % 10000) + 1
        text_marker.lifetime = Duration(seconds=1.0).to_msg()
        marker_array.markers.append(text_marker)

def main(args=None):
    rclpy.init(args=args)
    node = SemanticMapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()