#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
"""目标检测 + 气球颜色分类 ROS 节点

基于 det.py，增加 HSV 颜色分类功能：
  - YOLO 检测到 balloon 类别后，在检测框内用 HSV 分析颜色
  - class_name 输出带颜色后缀：red_balloon / blue_balloon / purple_balloon
  - 其他非 balloon 目标（如 hole / frame）保持原样

订阅话题:
    /camera/color/image_raw (sensor_msgs/Image): 输入图像
    /detect_ctrl (common_msgs/DetEnable): 检测控制命令

发布话题:
    /objects (common_msgs/Objects): 检测结果

Author: Object Detection Team
Date: 2026-03-17
"""

import os
import sys
import cv2
import numpy as np
import rospy
import torch
from datetime import datetime
from typing import Optional, Tuple

# 限制 PyTorch CPU 线程数，防止占满所有核心
# GPU 推理时 CPU 线程主要做预处理，2个线程已足够
torch.set_num_threads(2)
torch.set_num_interop_threads(1)

# ROS 消息类型
from sensor_msgs.msg import Image
from common_msgs.msg import Objects, Obj, DetEnable

# cv_bridge 用于特殊编码（但 Boost.Python 在 Python 3.9+ 有兼容问题，
# 对常见编码（rgb8/bgr8/mono8）优先用纯 NumPy 实现）
try:
    from cv_bridge import CvBridge as _CvBridge
    _CV_BRIDGE_AVAILABLE = True
except Exception:
    _CV_BRIDGE_AVAILABLE = False

# 设置路径以找到本地的 ultralytics 和 third_party
script_path = os.path.abspath(__file__)
script_dir = os.path.dirname(script_path)
pkg_dir = os.path.dirname(script_dir)
third_party_dir = os.path.join(pkg_dir, 'third_party')
clip_dir = os.path.join(third_party_dir, 'CLIP')

# 添加路径到 sys.path
for path in [third_party_dir, clip_dir, script_dir]:
    if path not in sys.path:
        sys.path.insert(0, path)

# 导入包模块
sys.path.insert(0, os.path.join(pkg_dir, 'src'))
from object_det.config import DetectionConfig
from object_det.detectors import YoloeDetector
from object_det.utils import DetectionVisualizer


# ============================================================
#  HSV 气球颜色分类（内联，不依赖外部模块）
# ============================================================
# HSV 阈值与 hsv_det.py 保持一致
_COLOR_RANGES = {
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
_MIN_PIXEL_COUNT = 30  # 低于此值认为该颜色不存在


def classify_balloon_color(
    image_bgr: np.ndarray,
    bbox: Tuple[int, int, int, int]
) -> Tuple[str, Optional[Tuple[int, int]]]:
    """对检测框内的气球进行 HSV 颜色分类

    Args:
        image_bgr: BGR 格式图像
        bbox: (x1, y1, x2, y2) 检测框坐标

    Returns:
        (color_name, center):
            color_name: "red" / "blue" / "purple" / "unknown"
            center: 该颜色区域的质心坐标 (cx, cy)，unknown 时为 None
    """
    x1, y1, x2, y2 = bbox
    roi_bgr = image_bgr[y1:y2, x1:x2]

    if roi_bgr.size == 0:
        return "unknown", None

    # 注意：image_bgr 在 is_simulation=False 时实际通道顺序是 RGB，
    # 对 YOLO 和可视化是对的，但 HSV 颜色分类需要用 RGB2HSV
    roi_hsv = cv2.cvtColor(roi_bgr, cv2.COLOR_RGB2HSV)

    best_color = "unknown"
    best_count = 0
    best_center = None
    counts = {}

    for color_name, ranges in _COLOR_RANGES.items():
        mask = np.zeros(roi_hsv.shape[:2], dtype=np.uint8)
        for r in ranges:
            mask |= cv2.inRange(roi_hsv, r["lower"], r["upper"])

        count = int(np.count_nonzero(mask))
        counts[color_name] = count

        if count > best_count and count >= _MIN_PIXEL_COUNT:
            best_count = count
            best_color = color_name

            coords = np.column_stack(np.where(mask > 0))
            if len(coords) > 0:
                cy_roi, cx_roi = np.mean(coords, axis=0).astype(int)
                best_center = (cx_roi + x1, cy_roi + y1)

    rospy.loginfo_throttle(2.0,
        f"[ColorClassify] red={counts.get('red',0)} "
        f"blue={counts.get('blue',0)} purple={counts.get('purple',0)} "
        f"-> {best_color}")

    return best_color, best_center


def _ros_image_to_numpy(msg: Image) -> np.ndarray:
    """纯 NumPy 实现的 ROS Image → OpenCV Mat 转换。

    避免使用 cv_bridge 的 C++ 扩展（依赖 Boost.Python，在 Python 3.9+ 有
    undefined symbol: _Py_fopen 问题）。

    支持常见编码：rgb8, bgr8, rgba8, bgra8, mono8, mono16, 16UC1, 32FC1
    其他编码降级使用 cv_bridge（如可用）。
    """
    enc = msg.encoding.lower()
    dtype_map = {
        "rgb8":   (np.uint8,   3),
        "bgr8":   (np.uint8,   3),
        "rgba8":  (np.uint8,   4),
        "bgra8":  (np.uint8,   4),
        "mono8":  (np.uint8,   1),
        "8uc1":   (np.uint8,   1),
        "mono16": (np.uint16,  1),
        "16uc1":  (np.uint16,  1),
        "32fc1":  (np.float32, 1),
    }

    if enc in dtype_map:
        dtype, channels = dtype_map[enc]
        arr = np.frombuffer(msg.data, dtype=dtype)
        if channels == 1:
            arr = arr.reshape(msg.height, msg.width)
        else:
            arr = arr.reshape(msg.height, msg.width, channels)
        # rgb8 → bgr8（OpenCV 内部用 BGR）
        if enc == "rgb8":
            arr = arr[:, :, ::-1].copy()
        elif enc == "rgba8":
            arr = cv2.cvtColor(arr, cv2.COLOR_RGBA2BGR)
        elif enc == "bgra8":
            arr = cv2.cvtColor(arr, cv2.COLOR_BGRA2BGR)
        return arr
    else:
        # 降级：尝试 cv_bridge
        if _CV_BRIDGE_AVAILABLE:
            rospy.logwarn_throttle(10.0,
                f"[_ros_image_to_numpy] 未知编码 '{msg.encoding}'，降级使用 cv_bridge")
            bridge = _CvBridge()
            return bridge.imgmsg_to_cv2(msg, "bgr8")
        raise ValueError(f"不支持的图像编码: {msg.encoding}，且 cv_bridge 不可用")


# ============================================================
#  检测节点
# ============================================================
class ObjectDetectionColorNode:
    """目标检测 + 气球颜色分类 ROS 节点

    与 det.py 功能相同，额外增加：
    - 当检测到 balloon 时，用 HSV 区分颜色
    - class_name 输出为 red_balloon / blue_balloon / purple_balloon
    """

    def __init__(self, config: Optional[DetectionConfig] = None):
        rospy.init_node('object_detection_color_node', anonymous=False)
        rospy.loginfo("Initializing Object Detection + Color Node...")

        if config is None:
            config = self._load_config_from_params()
        self.config = config
        rospy.loginfo(f"Configuration loaded:\n{config}")

        # 创建检测器
        self.detector = YoloeDetector(
            model_path=config.model_path,
            class_names=config.target_classes,
            conf_thres=config.conf_thres,
            use_half=config.use_half,
            device=None
        )

        # 动态解析目标类别 ID
        self.model_names = self.detector.names
        self.target_class_ids = []

        rospy.loginfo(f"Model loaded with {len(self.model_names)} classes.")

        name_to_id = {v: k for k, v in self.model_names.items()}
        for target_name in config.target_classes:
            if target_name in name_to_id:
                tid = name_to_id[target_name]
                self.target_class_ids.append(tid)
                rospy.loginfo(f"  Target '{target_name}' mapped to ID {tid}")
            else:
                rospy.logwarn(f"  Target class '{target_name}' NOT found in model! "
                             f"Available: {list(self.model_names.values())}")

        # 可视化器
        self.visualizer = DetectionVisualizer()
        self.bridge = None  # 不再使用 cv_bridge（Boost.Python 版本冲突）

        # 控制标志
        self._detection_enabled = True
        self.show_visualization = rospy.get_param('/object_detection/visualization/show_window', False)
        self._first_image_received = False
        self._cached_balloon_centers = {}

        # 准备图像保存目录
        if config.save_images:
            log_dir = rospy.get_param('/log_dir', "")
            if log_dir:
                self.save_dir = os.path.join(log_dir, "detection_images")
                try:
                    os.makedirs(self.save_dir, exist_ok=True)
                    rospy.loginfo(f"Image save directory: {self.save_dir}")
                except OSError as e:
                    rospy.logerr(f"Failed to create image save directory: {e}")
                    config.save_images = False
            else:
                rospy.logwarn("log_dir param not set, image saving disabled")
                config.save_images = False

        self._setup_ros_communication()
        rospy.loginfo("Object Detection + Color Node initialized!")

    # ---------- helpers ----------

    def _get_class_name(self, class_id: int) -> str:
        return self.model_names.get(class_id, f"class_{class_id}")

    def _is_target_class(self, class_id: int) -> bool:
        return class_id in self.target_class_ids

    def _load_config_from_params(self) -> DetectionConfig:
        ns = rospy.get_param('~param_namespace', 'object_detection')
        model_path = rospy.get_param(f'/{ns}/model/path', None)

        if model_path and model_path.startswith('$(find'):
            import rospkg
            rospack = rospkg.RosPack()
            pkg_name = model_path.split('$(find ')[1].split(')')[0]
            pkg_path = rospack.get_path(pkg_name)
            model_path = model_path.replace(f'$(find {pkg_name})', pkg_path)

        conf_thres = rospy.get_param(f'/{ns}/model/conf_threshold', 0.5)
        use_half = rospy.get_param(f'/{ns}/model/use_half', False)
        max_det = rospy.get_param(f'/{ns}/model/max_detections', 50)
        target_classes = rospy.get_param(f'/{ns}/detection/target_classes', ["hole", "balloon"])
        red_threshold = rospy.get_param(f'/{ns}/balloon_tracking/red_channel_threshold', 60)
        is_simulation = rospy.get_param(f'/{ns}/environment/is_simulation', True)
        save_images = rospy.get_param(f'/{ns}/image_saving/enabled', False)
        save_classes = rospy.get_param(f'/{ns}/image_saving/save_classes', [])

        rospy.loginfo(f"Loaded params from /{ns}")
        rospy.loginfo(f"  target_classes: {target_classes}")

        return DetectionConfig(
            model_path=model_path,
            conf_thres=conf_thres,
            use_half=use_half,
            max_det=max_det,
            target_classes=target_classes,
            red_channel_threshold=red_threshold,
            is_simulation=is_simulation,
            save_images=save_images,
            save_classes=save_classes
        )

    # ---------- ROS ----------

    def _setup_ros_communication(self):
        ns = rospy.get_param('~param_namespace', 'object_detection')
        image_topic = rospy.get_param(f'/{ns}/topics/image_input', '/camera/color/image_raw')
        objects_topic = rospy.get_param(f'/{ns}/topics/detection_output', '/objects')
        control_topic = rospy.get_param(f'/{ns}/topics/control_input', '/detect_ctrl')

        self.image_sub = rospy.Subscriber(image_topic, Image, self._image_callback, queue_size=1)
        self.control_sub = rospy.Subscriber(control_topic, DetEnable, self._control_callback, queue_size=10)
        self.objects_pub = rospy.Publisher(objects_topic, Objects, queue_size=10)

        rospy.loginfo(f"Subscribed to: {image_topic}")
        rospy.loginfo(f"Publishing to: {objects_topic}")

    def _control_callback(self, msg: DetEnable):
        self._detection_enabled = msg.is_open
        rospy.loginfo(f"Detection {'enabled' if self._detection_enabled else 'disabled'}")

    # ---------- 主回调 ----------

    def _image_callback(self, msg: Image):
        import time
        start_time = time.time()
        self._cached_balloon_centers.clear()

        if not self._first_image_received:
            rospy.loginfo(f"Received first image! Size: {msg.width}x{msg.height}")
            self._first_image_received = True

        if not self._detection_enabled:
            return

        # 转换图像（使用纯 NumPy，避免 Boost.Python 版本不兼容问题）
        try:
            image_raw = _ros_image_to_numpy(msg)
            if self.config.is_simulation:
                image_bgr = image_raw
            else:
                image_bgr = image_raw[:, :, ::-1]
            image_rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
        except Exception as e:
            rospy.logerr(f"Failed to convert image: {e}")
            return

        # YOLO 检测
        detections = self.detector.predict(image_rgb)

        if detections is not None and len(detections) > 0:
            objects_msg = self._process_detections(detections, image_bgr, msg.header)
            self.objects_pub.publish(objects_msg)
            rospy.logdebug(f"Detected {len(detections)} objects")

        processing_time_ms = (time.time() - start_time) * 1000.0

        if self.show_visualization:
            self._visualize(image_bgr, detections, processing_time_ms)

    # ---------- 处理检测结果（核心改动在这里） ----------

    def _process_detections(self, detections, image_bgr, header) -> Objects:
        objects_msg = Objects()
        objects_msg.header = header
        objects_msg.source_id = 0

        for det in detections:
            x1, y1, x2, y2, conf, cls_id = det
            cls_id = int(cls_id)

            if not self._is_target_class(cls_id):
                continue

            bbox = (int(x1), int(y1), int(x2), int(y2))
            obj = Obj()
            obj.class_name = self._get_class_name(cls_id)
            obj.left_top_x = bbox[0]
            obj.left_top_y = bbox[1]
            obj.right_bottom_x = bbox[2]
            obj.right_bottom_y = bbox[3]
            obj.score = float(conf)
            obj.header = header

            # ===== 关键改动：balloon 类别 → 颜色分类 =====
            # 模型类别可能是 "balloon" 或 "red_balloon" 等，只要包含 balloon 就做颜色分类
            if "balloon" in obj.class_name:
                color, center = classify_balloon_color(image_bgr, bbox)

                # 用颜色后缀替换类名
                if color != "unknown":
                    obj.class_name = color + "_balloon"
                    rospy.logdebug("Balloon color: %s", color)
                else:
                    # 颜色无法判断，保留通用名
                    rospy.logwarn_throttle(5.0, "Cannot classify balloon color, keeping 'balloon'")

                # 用颜色区域质心作为中心点（比 bbox 中心更精确）
                if center is not None:
                    obj.center_x = center[0]
                    obj.center_y = center[1]
                else:
                    obj.center_x = (bbox[0] + bbox[2]) // 2
                    obj.center_y = (bbox[1] + bbox[3]) // 2

                self._cached_balloon_centers[len(objects_msg.objects)] = (obj.center_x, obj.center_y)
            else:
                # 非 balloon 目标，使用 bbox 中心
                obj.center_x = (bbox[0] + bbox[2]) // 2
                obj.center_y = (bbox[1] + bbox[3]) // 2

            objects_msg.objects.append(obj)

        return objects_msg

    # ---------- 可视化 ----------

    def _visualize(self, image, detections, processing_time_ms=0.0):
        img_vis = image.copy()

        # 颜色映射（可视化用）
        color_map = {
            "red_balloon": (0, 0, 255),
            "blue_balloon": (255, 0, 0),
            "purple_balloon": (255, 0, 255),
        }

        if detections is not None and len(detections) > 0:
            balloon_idx = 0
            for det in detections:
                x1, y1, x2, y2, conf, cls_id = det
                cls_id = int(cls_id)

                if not self._is_target_class(cls_id):
                    continue

                label = self._get_class_name(cls_id)

                # 对 balloon 重新做颜色分类以获取带颜色的标签（用于可视化）
                display_label = label
                vis_color = None
                if "balloon" in label:
                    bbox = (int(x1), int(y1), int(x2), int(y2))
                    color, _ = classify_balloon_color(image, bbox)
                    if color != "unknown":
                        display_label = color + "_balloon"
                        vis_color = color_map.get(display_label)

                # 绘制检测框
                img_vis = self.visualizer.draw_detection(
                    img_vis,
                    (int(x1), int(y1), int(x2), int(y2)),
                    display_label,
                    float(conf)
                )

                # 如果是 balloon，绘制颜色质心
                if "balloon" in label:
                    center = self._cached_balloon_centers.get(balloon_idx)
                    if center is not None:
                        pt_color = vis_color if vis_color else (0, 255, 0)
                        cv2.circle(img_vis, center, 6, pt_color, -1)
                        cv2.circle(img_vis, center, 8, (255, 255, 255), 1)
                    balloon_idx += 1

        # 显示
        img_vis_rgb = cv2.cvtColor(img_vis, cv2.COLOR_BGR2RGB)

        try:
            if not hasattr(self, '_window_created'):
                cv2.namedWindow('Detection + Color', cv2.WINDOW_NORMAL)
                cv2.resizeWindow('Detection + Color', 640, 480)
                self._window_created = True

            cv2.imshow('Detection + Color', img_vis_rgb)
            cv2.waitKey(2)
        except Exception as e:
            rospy.logwarn_throttle(5.0, f"Display error: {e}")

    # ---------- 运行 ----------

    def run(self):
        rospy.loginfo("Object Detection + Color Node running (event-driven)...")
        try:
            rospy.spin()
        except KeyboardInterrupt:
            rospy.loginfo("Shutting down...")
        finally:
            cv2.destroyAllWindows()


def main():
    try:
        node = ObjectDetectionColorNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Fatal error: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()
