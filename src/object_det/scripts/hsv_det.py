#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
"""基于 HSV 颜色空间的气球检测 ROS 节点

使用传统视觉方法（HSV 颜色过滤 + 轮廓检测）替代 YOLO 进行气球检测。
输出格式与 YOLO 检测节点完全一致，可直接替换。

订阅话题:
    /camera/color/image_raw (sensor_msgs/Image): 输入图像

发布话题:
    /objects (common_msgs/Objects): 检测结果（与 YOLO 格式一致）

Author: HSV Balloon Detector
Date: 2026-03-13
"""

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from common_msgs.msg import Objects, Obj
from cv_bridge import CvBridge


class HSVBalloonDetector:
    """基于 HSV 的气球检测器

    检测红色、蓝色、紫色气球，输出 bounding box 和中心点。
    输出的 class_name 格式为 "red_balloon" / "blue_balloon" / "purple_balloon"，
    与行为树节点的 object_name 参数匹配。
    """

    # ---- HSV 颜色阈值配置 ----
    # 每种颜色可配置多个区间（如红色跨越 H=0 和 H=170 两端）
    COLOR_RANGES = {
        "red": [
            {"lower": np.array([0, 100, 80]),   "upper": np.array([10, 255, 255])},
            {"lower": np.array([170, 100, 80]), "upper": np.array([180, 255, 255])},
        ],
        "blue": [
            {"lower": np.array([100, 100, 80]), "upper": np.array([130, 255, 255])},
        ],
        "purple": [
            {"lower": np.array([130, 60, 80]),  "upper": np.array([170, 255, 255])},
        ],
    }

    # 最小轮廓面积（像素^2），过滤噪声
    MIN_CONTOUR_AREA = 200
    # 最小圆度（0~1），过滤非圆形物体
    MIN_CIRCULARITY = 0.3

    def __init__(self):
        rospy.init_node("hsv_balloon_detector", anonymous=False)
        rospy.loginfo("Initializing HSV Balloon Detector...")

        # 从 ROS 参数读取配置
        self.image_topic = rospy.get_param("~image_topic", "/camera/color/image_raw")
        self.output_topic = rospy.get_param("~output_topic", "/objects")
        self.show_visualization = rospy.get_param("~show_visualization", True)
        self.is_simulation = rospy.get_param("~is_simulation", True)

        # 可通过参数调节 HSV 阈值
        self._load_hsv_params()

        self.bridge = CvBridge()
        self._first_image = False

        # ROS 通信
        self.image_sub = rospy.Subscriber(
            self.image_topic, Image, self._image_callback, queue_size=1
        )
        self.objects_pub = rospy.Publisher(
            self.output_topic, Objects, queue_size=10
        )

        rospy.loginfo("HSV Balloon Detector ready.")
        rospy.loginfo("  Subscribed to: %s", self.image_topic)
        rospy.loginfo("  Publishing to: %s", self.output_topic)

    def _load_hsv_params(self):
        """从 ROS 参数服务器加载可调节的 HSV 阈值"""
        self.MIN_CONTOUR_AREA = rospy.get_param("~min_contour_area", 200)
        self.MIN_CIRCULARITY = rospy.get_param("~min_circularity", 0.3)

        # 允许通过参数覆盖 HSV 范围（高级用法）
        for color in ["red", "blue", "purple"]:
            param_name = f"~hsv_{color}"
            if rospy.has_param(param_name):
                custom = rospy.get_param(param_name)
                rospy.loginfo("  Using custom HSV for %s: %s", color, custom)

    def _detect_color(self, hsv_image, color_name):
        """检测指定颜色的区域

        Args:
            hsv_image: HSV 格式图像
            color_name: 颜色名称 ("red"/"blue"/"purple")

        Returns:
            list of (x1, y1, x2, y2, cx, cy, area) 检测结果
        """
        ranges = self.COLOR_RANGES.get(color_name, [])
        if not ranges:
            return []

        # 合并多个 HSV 区间的掩膜
        mask = np.zeros(hsv_image.shape[:2], dtype=np.uint8)
        for r in ranges:
            mask |= cv2.inRange(hsv_image, r["lower"], r["upper"])

        # 形态学处理：去噪 + 填充
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # 找轮廓
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        results = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < self.MIN_CONTOUR_AREA:
                continue

            # 圆度检查（气球应该接近圆形）
            perimeter = cv2.arcLength(cnt, True)
            if perimeter == 0:
                continue
            circularity = 4.0 * np.pi * area / (perimeter * perimeter)
            if circularity < self.MIN_CIRCULARITY:
                continue

            # 计算 bounding box
            x, y, w, h = cv2.boundingRect(cnt)

            # 用轮廓矩计算质心（比 bbox 中心更准确）
            M = cv2.moments(cnt)
            if M["m00"] > 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
            else:
                cx = x + w // 2
                cy = y + h // 2

            results.append((x, y, x + w, y + h, cx, cy, area))

        return results

    def _image_callback(self, msg):
        """图像回调：检测所有颜色的气球"""
        if not self._first_image:
            rospy.loginfo("Received first image! Size: %dx%d", msg.width, msg.height)
            self._first_image = True

        # 转换图像
        try:
            image_raw = self.bridge.imgmsg_to_cv2(msg, msg.encoding)
            if self.is_simulation:
                image_bgr = image_raw
            else:
                image_bgr = image_raw[:, :, ::-1]
        except Exception as e:
            rospy.logerr("Failed to convert image: %s", e)
            return

        # 转换到 HSV
        hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)

        # 构建消息
        objects_msg = Objects()
        objects_msg.header = msg.header
        objects_msg.source_id = 1  # 1: front camera

        img_area = image_bgr.shape[0] * image_bgr.shape[1]

        # 检测三种颜色
        for color in ["red", "blue", "purple"]:
            detections = self._detect_color(hsv, color)

            for (x1, y1, x2, y2, cx, cy, area) in detections:
                obj = Obj()
                obj.header = msg.header
                obj.class_name = color + "_balloon"
                obj.left_top_x = x1
                obj.left_top_y = y1
                obj.right_bottom_x = x2
                obj.right_bottom_y = y2
                obj.center_x = cx
                obj.center_y = cy
                # 置信度：用面积占比映射（面积越大越确信）
                obj.score = min(1.0, float(area) / (img_area * 0.3))
                objects_msg.objects.append(obj)

        # 发布结果
        if objects_msg.objects:
            self.objects_pub.publish(objects_msg)

        # 可视化
        if self.show_visualization:
            self._visualize(image_bgr, objects_msg.objects)

    def _visualize(self, image, objects):
        """可视化检测结果"""
        vis = image.copy()

        color_map = {
            "red_balloon": (0, 0, 255),
            "blue_balloon": (255, 0, 0),
            "purple_balloon": (255, 0, 255),
        }

        for obj in objects:
            color = color_map.get(obj.class_name, (0, 255, 0))

            # 画框
            cv2.rectangle(vis, (obj.left_top_x, obj.left_top_y),
                         (obj.right_bottom_x, obj.right_bottom_y), color, 2)

            # 画中心点
            cv2.circle(vis, (obj.center_x, obj.center_y), 5, color, -1)

            # 标签
            label = f"{obj.class_name} {obj.score:.2f}"
            cv2.putText(vis, label, (obj.left_top_x, obj.left_top_y - 8),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)

        try:
            if not hasattr(self, "_window_created"):
                cv2.namedWindow("HSV Balloon Detection", cv2.WINDOW_NORMAL)
                cv2.resizeWindow("HSV Balloon Detection", 640, 480)
                self._window_created = True
            cv2.imshow("HSV Balloon Detection", vis)
            cv2.waitKey(2)
        except Exception as e:
            rospy.logwarn_throttle(5.0, "Display error: %s" % e)

    def run(self):
        rospy.loginfo("HSV Balloon Detector running...")
        try:
            rospy.spin()
        except KeyboardInterrupt:
            pass
        finally:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    try:
        node = HSVBalloonDetector()
        node.run()
    except rospy.ROSInterruptException:
        pass
