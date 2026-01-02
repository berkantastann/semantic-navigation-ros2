#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from rclpy.duration import Duration
from rclpy.time import Time

from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PointStamped

from cv_bridge import CvBridge
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

        # ---------------- TF ----------------
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ---------------- MODEL ----------------
        pkg_dir = get_package_share_directory('semantic_nav')
        model_path = os.path.join(pkg_dir, 'models', 'yolov8n.onnx')
        self.model = YOLO(model_path, task='detect')

        # ---------------- DB ----------------
        self.db_path = os.path.expanduser("~/Desktop/robot_ws/src/semantic_nav/db/semantic_db.json")
        self.detected_objects = []

        if os.path.exists(self.db_path):
            try:
                with open(self.db_path, 'r') as f:
                    self.detected_objects = json.load(f)

                for obj in self.detected_objects:
                    if "count" not in obj:
                        obj["count"] = 1

                self.get_logger().info(
                    f"📦 DB yüklendi ({len(self.detected_objects)} nesne)"
                )
            except Exception as e:
                self.get_logger().warn(f"DB okunamadı: {e}")

        # ---------------- SUBSCRIBERS ----------------
        self.rgb_sub = message_filters.Subscriber(
            self, Image, '/camera/image',
            qos_profile=qos_profile_sensor_data)

        self.depth_sub = message_filters.Subscriber(
            self, Image, '/camera/depth_image',
            qos_profile=qos_profile_sensor_data)

        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub], 10, 0.3)
        self.ts.registerCallback(self.sync_callback)

        self.create_subscription(
            CameraInfo, '/camera/camera_info',
            self.info_callback, qos_profile_sensor_data)

        # ---------------- MARKER PUBLISHER (RViz FIX) ----------------
        marker_qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        self.marker_pub = self.create_publisher(
            MarkerArray, '/semantic_markers', marker_qos)

        self.br = CvBridge()

        # ---------------- CAMERA PARAMS ----------------
        self.fx = None

        # ---------------- PERFORMANCE ----------------
        self.frame_counter = 0

        # Node açılır açılmaz markerları bas
        self.create_timer(1.0, self.publish_all_markers)

        self.get_logger().info("✅ Semantic Mapper ÇALIŞIYOR (RViz Garantili)")


    def info_callback(self, msg):
        if self.fx is None:
            self.fx = msg.k[0]
            self.fy = msg.k[4]
            self.cx = msg.k[2]
            self.cy = msg.k[5]
            self.width = msg.width


    def sync_callback(self, rgb_msg, depth_msg):
        if self.fx is None:
            return

        self.frame_counter += 1
        if self.frame_counter % 5 != 0:
            return

        try:
            rgb = self.br.imgmsg_to_cv2(rgb_msg, 'bgr8')
            try:
                depth = self.br.imgmsg_to_cv2(depth_msg, '32FC1')
            except:
                depth = self.br.imgmsg_to_cv2(depth_msg, 'passthrough')

            results = self.model.predict(rgb, conf=0.6, verbose=False)

            target_frame = "camera_depth_optical_frame"

            if not self.tf_buffer.can_transform(
                "map", target_frame, Time(), Duration(seconds=0.2)):
                return

            transform = self.tf_buffer.lookup_transform(
                "map", target_frame, Time(), Duration(seconds=0.2))

            db_changed = False

            for r in results:
                for box in r.boxes:
                    cls_id = int(box.cls[0])
                    name = self.model.names[cls_id]

                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    u = int((x1 + x2) / 2)
                    v = int((y1 + y2) / 2)

                    margin = self.width * 0.2
                    if u < margin or u > self.width - margin:
                        continue

                    roi = depth[max(0, v-3):min(depth.shape[0], v+3),
                                max(0, u-3):min(depth.shape[1], u+3)]
                    if roi.size == 0:
                        continue

                    z = np.nanmedian(roi)
                    if np.isnan(z) or z < 0.4 or z > 2.5:
                        continue

                    x_cam = (u - self.cx) * z / self.fx
                    y_cam = (v - self.cy) * z / self.fy

                    p = PointStamped()
                    p.header.frame_id = target_frame
                    p.header.stamp = Time().to_msg()
                    p.point.x = float(x_cam)
                    p.point.y = float(y_cam)
                    p.point.z = float(z)

                    p_map = tf2_geometry_msgs.do_transform_point(p, transform)
                    p_map.point.z = 0.0

                    self.update_database(name, p_map.point.x, p_map.point.y)
                    db_changed = True

            if db_changed or self.frame_counter % 15 == 0:
                self.cleanup_duplicates()
                self.flush_to_disk()
                self.publish_all_markers()

        except Exception as e:
            self.get_logger().error(f"Hata: {e}")


    def update_database(self, name, x, y):
        MATCH_DISTANCE = 1.2
        ALPHA = 0.2

        for obj in self.detected_objects:
            if obj['name'] == name:
                dist = np.hypot(obj['x'] - x, obj['y'] - y)
                if dist < MATCH_DISTANCE:
                    obj['x'] = (1 - ALPHA) * obj['x'] + ALPHA * x
                    obj['y'] = (1 - ALPHA) * obj['y'] + ALPHA * y
                    obj['count'] += 1
                    return

        self.detected_objects.append({
            "name": name,
            "x": x,
            "y": y,
            "count": 1
        })


    def cleanup_duplicates(self):
        MERGE_RADIUS = 0.8
        cleaned = []

        for obj in self.detected_objects:
            merged = False
            for c in cleaned:
                if c['name'] == obj['name']:
                    d = np.hypot(c['x'] - obj['x'], c['y'] - obj['y'])
                    if d < MERGE_RADIUS:
                        total = c['count'] + obj['count']
                        c['x'] = (c['x'] * c['count'] + obj['x'] * obj['count']) / total
                        c['y'] = (c['y'] * c['count'] + obj['y'] * obj['count']) / total
                        c['count'] = total
                        merged = True
                        break
            if not merged:
                cleaned.append(obj)

        self.detected_objects = cleaned


    def flush_to_disk(self):
        try:
            with open(self.db_path, 'w') as f:
                json.dump(self.detected_objects, f, indent=2)
        except:
            pass


    def publish_all_markers(self):
        if not self.detected_objects:
            return

        arr = MarkerArray()

        clear = Marker()
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)

        for i, obj in enumerate(self.detected_objects):
            m = Marker()
            m.header.frame_id = "map"
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = "semantic_objects"
            m.id = i * 2
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = obj['x']
            m.pose.position.y = obj['y']
            m.pose.position.z = 0.2
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.3

            conf = min(1.0, obj['count'] / 20.0)
            m.color.r = 1.0 - conf
            m.color.g = conf
            m.color.b = 0.0
            m.color.a = 0.9
            arr.markers.append(m)

            t = Marker()
            t.header.frame_id = "map"
            t.header.stamp = self.get_clock().now().to_msg()
            t.ns = "semantic_labels"
            t.id = i * 2 + 1
            t.type = Marker.TEXT_VIEW_FACING
            t.action = Marker.ADD
            t.pose.position.x = obj['x']
            t.pose.position.y = obj['y']
            t.pose.position.z = 0.6
            t.scale.z = 0.25
            t.color.a = 1.0
            t.color.r = t.color.g = t.color.b = 1.0
            t.text = f"{obj['name']} ({obj['count']})"
            arr.markers.append(t)

        self.marker_pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = SemanticMapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()