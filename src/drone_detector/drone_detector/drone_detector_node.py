#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
YOLOv8 无人机识别节点：
订阅相机图像话题，使用 src/best_v8.pt 模型进行推理，发布 2D 检测结果与可选的可视化图像。
"""

import os
import sys

# 在 conda 等环境下 PYTHONPATH 可能未正确传递，显式加入 ROS2 的 site-packages 以便找到 vision_msgs
def _ensure_ros_python_path():
    try:
        import vision_msgs  # noqa: F401
        return
    except ImportError:
        pass
    for key in ('PYTHONPATH', 'AMENT_PREFIX_PATH'):
        for path in os.environ.get(key, '').split(os.pathsep):
            path = path.strip()
            if not path:
                continue
            # AMENT_PREFIX_PATH 下各 prefix 的 lib/python3.x/site-packages
            if key == 'AMENT_PREFIX_PATH':
                for sub in ('lib/python3.10/site-packages', 'lib/python3.8/site-packages'):
                    p = os.path.join(path, sub)
                    if os.path.isdir(p) and p not in sys.path:
                        sys.path.insert(0, p)
            elif os.path.isdir(path) and path not in sys.path:
                sys.path.insert(0, path)
    # 若仍无 vision_msgs，尝试 Humble 默认路径
    try:
        import vision_msgs  # noqa: F401
        return
    except ImportError:
        pass
    ros_lib = '/opt/ros/humble/lib/python3.10/site-packages'
    if os.path.isdir(ros_lib) and ros_lib not in sys.path:
        sys.path.insert(0, ros_lib)


_ensure_ros_python_path()

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from vision_msgs.msg import BoundingBox2D
from std_msgs.msg import Header
from cv_bridge import CvBridge
import cv2
import numpy as np


class DroneDetectorNode(Node):
    def __init__(self):
        super().__init__('drone_detector_node')

        self.declare_parameter('model_path', 'model/best_v8.pt')
        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('conf_threshold', 0.25)
        self.declare_parameter('publish_annotated', True)
        self.declare_parameter('device', '')  # '' 表示自动选择，可选 'cuda:0' 或 'cpu'

        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.conf_threshold = self.get_parameter('conf_threshold').get_parameter_value().double_value
        try:
            self.publish_annotated = self.get_parameter('publish_annotated').get_parameter_value().bool_value
        except Exception:
            s = self.get_parameter('publish_annotated').get_parameter_value().string_value
            self.publish_annotated = s.lower() in ('1', 'true', 'yes')
        device = self.get_parameter('device').get_parameter_value().string_value

        # 解析模型路径：支持相对路径（相对于工作空间根目录 ros2_air）
        if not os.path.isabs(model_path):
            # 尝试从 COLCON_PREFIX_PATH 推断工作空间，或使用 cwd
            ws = os.environ.get('COLCON_PREFIX_PATH', '').split(os.pathsep)[0]
            if ws:
                # COLCON_PREFIX_PATH 通常是 <ws>/install
                base = os.path.dirname(ws)
                if os.path.basename(ws) != 'install' and os.path.isdir(os.path.join(ws, 'src')):
                    base = ws
            else:
                base = os.getcwd()
            model_path = os.path.join(base, model_path)
        if not os.path.isfile(model_path):
            self.get_logger().error('模型文件不存在: %s' % model_path)
            raise FileNotFoundError('模型文件不存在: %s' % model_path)

        try:
            from ultralytics import YOLO
            self.model = YOLO(model_path)
            self.get_logger().info('已加载 YOLOv8 模型: %s' % model_path)
        except Exception as e:
            self.get_logger().error('加载 YOLOv8 失败，请安装: pip install ultralytics. 错误: %s' % e)
            raise

        self.bridge = CvBridge()
        self.sub_image = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            10
        )
        self.pub_detections = self.create_publisher(
            Detection2DArray,
            'drone_detections',
            10
        )
        if self.publish_annotated:
            self.pub_image = self.create_publisher(
                Image,
                'drone_detection_image',
                10
            )
        self.get_logger().info('订阅图像: %s，发布检测: drone_detections' % image_topic)

    def image_callback(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn('cv_bridge 转换失败: %s' % e)
            return

        results = self.model.predict(
            source=cv_image,
            conf=self.conf_threshold,
            verbose=False,
            stream=False
        )

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = msg.header.frame_id if msg.header.frame_id else 'camera_optical_frame'

        detections_msg = Detection2DArray()
        detections_msg.header = header

        if not results:
            self.pub_detections.publish(detections_msg)
            if self.publish_annotated:
                self.pub_image.publish(self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8'))
            return

        result = results[0]
        if result.boxes is None:
            self.pub_detections.publish(detections_msg)
            if self.publish_annotated:
                self.pub_image.publish(self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8'))
            return

        h, w = cv_image.shape[:2]
        for box in result.boxes:
            xyxy = box.xyxy[0].cpu().numpy()
            conf = float(box.conf[0].cpu().numpy())
            cls_id = int(box.cls[0].cpu().numpy())
            cls_name = result.names.get(cls_id, 'object')

            x1, y1, x2, y2 = xyxy
            cx = (x1 + x2) / 2.0
            cy = (y1 + y2) / 2.0
            sx = x2 - x1
            sy = y2 - y1

            det = Detection2D()
            det.header = header
            det.bbox = BoundingBox2D()
            det.bbox.center.position.x = float(cx)
            det.bbox.center.position.y = float(cy)
            det.bbox.center.theta = 0.0
            det.bbox.size_x = float(sx)
            det.bbox.size_y = float(sy)
            hyp_wp = ObjectHypothesisWithPose()
            hyp_wp.hypothesis.class_id = cls_name
            hyp_wp.hypothesis.score = conf
            # pose 留默认值即可（此处是 2D 检测，不提供 6D 位姿）
            det.results = [hyp_wp]
            detections_msg.detections.append(det)

            if self.publish_annotated:
                cv2.rectangle(cv_image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                label = '%s %.2f' % (cls_name, conf)
                cv2.putText(cv_image, label, (int(x1), int(y1) - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        self.pub_detections.publish(detections_msg)
        if self.publish_annotated:
            out_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            out_msg.header = header
            self.pub_image.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DroneDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
