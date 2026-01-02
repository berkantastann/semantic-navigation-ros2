#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan, Image
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import json
import os
import threading
import time
import cv2
from cv_bridge import CvBridge
import math
import sys

class SemanticSensorNode(Node):
    def __init__(self):
        super().__init__('semantic_sensor_node')
        
        self.min_front_distance = 10.0
        self.latest_image = None
        self.bridge = CvBridge()

        self.create_subscription(LaserScan, '/scan', self.laser_callback, qos_profile_sensor_data)
        self.create_subscription(Image, '/camera/image', self.camera_callback, qos_profile_sensor_data)

    def laser_callback(self, msg: LaserScan):
        mid = len(msg.ranges) // 2
        window = 20
        front = msg.ranges[mid - window : mid + window]
        
        valid = [r for r in front if not math.isinf(r) and not math.isnan(r) and r > 0.0]
        if valid:
            self.min_front_distance = min(valid)
        else:
            self.min_front_distance = 10.0

    def camera_callback(self, msg):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except:
            pass

def main():
    rclpy.init()
    
    sensor_node = SemanticSensorNode()
    executor = MultiThreadedExecutor()
    executor.add_node(sensor_node)
    
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    navigator = BasicNavigator()
    
    db_path = os.path.expanduser("~/Desktop/robot_ws/src/semantic_nav/db/semantic_db.json")
    objects_db = {}
    
    if os.path.exists(db_path):
        try:
            with open(db_path, 'r') as f:
                data = json.load(f)
                for obj in data:
                    objects_db[obj['name']] = obj
            print(f"📦 DB Yüklendi: {len(objects_db)} nesne.")
        except:
            print("❌ DB okunamadı!")

    print("⏳ Nav2 bekleniyor (Active)...")
    navigator.waitUntilNav2Active()
    print("✅ SİSTEM HAZIR! Komut bekleniyor.")

    while rclpy.ok():
        try:
            target_name = input("\nGitmek istediğiniz nesne (Çıkış için 'q'): ").strip()
            if target_name.lower() == 'q':
                break
            
            if target_name not in objects_db:
                print(f"❌ '{target_name}' bulunamadı! Mevcut: {list(objects_db.keys())}")
                continue
            
            obj_data = objects_db[target_name]
            tx, ty = obj_data['x'], obj_data['y']
            
            print(f"🚀 {target_name} hedefine gidiliyor... (X:{tx:.2f}, Y:{ty:.2f})")
            
            goal = PoseStamped()
            goal.header.frame_id = 'map'
            goal.header.stamp = navigator.get_clock().now().to_msg()
            goal.pose.position.x = tx
            goal.pose.position.y = ty
            goal.pose.orientation.w = 1.0
            
            navigator.goToPose(goal)
            
            manual_success = False

            while not navigator.isTaskComplete():
                feedback = navigator.getFeedback()
                lidar_dist = sensor_node.min_front_distance
                
                if feedback:
                    remain = feedback.distance_remaining
                    print(f"Kalan: {remain:.2f}m | Ön Engel: {lidar_dist:.2f}m   ", end='\r')
                    
                    if remain < 0.8:
                        print(f"\n🛑 Hedefe varıldı ({target_name}). Fren yapılıyor.")
                        navigator.cancelTask()
                        manual_success = True
                        break
                    
                    if lidar_dist < 0.5:
                        print(f"\n⚠️ ACİL DURUŞ! Önümde engel var ({lidar_dist:.2f}m).")
                        navigator.cancelTask()
                        manual_success = True 
                        break
                
                time.sleep(0.1)
            
            result = navigator.getResult()
            
            if manual_success or result == TaskResult.SUCCEEDED or result == TaskResult.CANCELED:
                print(f"✅ {target_name} operasyonu tamamlandı.")

                if sensor_node.latest_image is not None:
                    print("📸 Kamera açılıyor (Kapatmak için bir tuşa bas)...")
                    cv2.imshow(f"HEDEF: {target_name}", sensor_node.latest_image)
                    cv2.waitKey(0)
                    cv2.destroyAllWindows()
                else:
                    print("⚠️ Kamera görüntüsü alınamadı.")
            else:
                print("❌ Rota başarısız oldu!")
                
        except KeyboardInterrupt:
            break
        except Exception as e:
            print(f"Hata: {e}")

    sensor_node.destroy_node()
    rclpy.shutdown()
    print("\nSistem kapatıldı.")

if __name__ == '__main__':
    main()