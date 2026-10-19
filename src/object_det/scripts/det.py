#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
"""目标检测 ROS 节点

该节点使用 YOLOE 模型检测图像中的 hole 和 balloon 目标。
对于 balloon 目标，会额外进行红色像素追踪以获得更精确的中心点。

订阅话题:
    /camera/color/image_raw (sensor_msgs/Image): 输入图像
    /detect_ctrl (common_msgs/DetEnable): 检测控制命令

发布话题:
    /objects (common_msgs/Objects): 检测结果

Author: Object Detection Team
Date: 2026-01-20
"""

import os
import sys
import cv2
import numpy as np
import rospy
from datetime import datetime
from typing import Optional

# ROS 消息类型
from sensor_msgs.msg import Image
from common_msgs.msg import Objects, Obj, DetEnable
from cv_bridge import CvBridge

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
from object_det.trackers import BalloonTracker
from object_det.utils import DetectionVisualizer


class ObjectDetectionNode:
    """目标检测 ROS 节点
    
    该节点负责：
    1. 订阅相机图像（事件驱动，收到图像即处理）
    2. 使用 YOLOE 模型进行目标检测
    3. 对 balloon 进行红色像素追踪
    4. 发布检测结果
    5. 可视化检测结果
    
    Attributes:
        config: 检测配置
        detector: YOLOE 检测器
        balloon_tracker: 气球追踪器
        visualizer: 可视化器
        bridge: ROS-OpenCV 桥接器
        show_visualization: 是否显示可视化窗口
    """
    
    def __init__(self, config: Optional[DetectionConfig] = None):
        """初始化检测节点
        
        Args:
            config: 检测配置，如果为 None 则从 ROS 参数服务器读取
        """
        # 初始化 ROS 节点
        rospy.init_node('object_detection_node', anonymous=False)
        rospy.loginfo("Initializing Object Detection Node...")
        
        # 从 ROS 参数服务器加载配置
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
            device=None  # 自动检测
        )
        
        # 动态解析目标类别 ID
        self.model_names = self.detector.names
        self.target_class_ids = []
        
        rospy.loginfo(f"Model loaded with {len(self.model_names)} classes.")
        
        # 建立名称到ID的映射
        name_to_id = {v: k for k, v in self.model_names.items()}
        
        for target_name in config.target_classes:
            if target_name in name_to_id:
                tid = name_to_id[target_name]
                self.target_class_ids.append(tid)
                rospy.loginfo(f"  Target '{target_name}' mapped to ID {tid}")
            else:
                rospy.logwarn(f"  Target class '{target_name}' NOT found in model! Available classes: {list(self.model_names.values())}")
        
        # 创建气球追踪器
        self.balloon_tracker = BalloonTracker(
            red_threshold=config.red_channel_threshold
        )

        # 准备图像保存目录（使用统一日志目录）
        if config.save_images:
            # 从 ROS 参数服务器读取统一日志目录
            log_dir = rospy.get_param('/log_dir', "$(optenv LOG_DIR /root/28com_ws/logs/image)")
            
            if log_dir:
                # 使用统一日志目录下的 detection_images 子目录
                self.save_dir = os.path.join(log_dir, "detection_images")
                try:
                    os.makedirs(self.save_dir, exist_ok=True)
                    rospy.loginfo(f"Image save directory: {self.save_dir}")
                except OSError as e:
                    rospy.logerr(f"Failed to create image save directory: {e}")
                    config.save_images = False
            else:
                # log_dir 未设置，禁用图片保存
                rospy.logwarn("log_dir param not set, image saving disabled")
                config.save_images = False
        
        # 创建可视化器
        self.visualizer = DetectionVisualizer()
        
        # ROS 桥接器
        self.bridge = CvBridge()
        
        # 控制标志
        self._detection_enabled = True
        self.show_visualization = rospy.get_param(
            '~visualization/show_window', 
            True
        )
        self._first_image_received = False  # 标志位：是否已收到第一帧图像
        self._cached_balloon_centers = {}  # 缓存气球中心点，避免重复计算
             
        # 设置 ROS 通信
        self._setup_ros_communication()
        
        rospy.loginfo("Object Detection Node initialized successfully!")

    def _get_class_name(self, class_id: int) -> str:
        """根据类别 ID 获取类别名称 (动态)"""
        if class_id in self.model_names:
            return self.model_names[class_id]
        return f"class_{class_id}"

    def _is_target_class(self, class_id: int) -> bool:
        """判断是否为目标类别 (动态)"""
        return class_id in self.target_class_ids
    
    def _load_config_from_params(self) -> DetectionConfig:
        """从 ROS 参数服务器加载配置
        
        Returns:
            DetectionConfig 对象
        """
        # 获取参数命名空间
        ns = rospy.get_param('~param_namespace', 'object_detection')
        
        # 读取配置参数
        model_path = rospy.get_param(f'/{ns}/model/path', None)
        if model_path and model_path.startswith('$(find'):
            # 处理 $(find package) 语法
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

        # 图像保存参数
        save_images = rospy.get_param(f'/{ns}/image_saving/enabled', False)
        save_classes = rospy.get_param(f'/{ns}/image_saving/save_classes', [])
        
        rospy.loginfo(f"Loaded parameters from namespace: /{ns}")
        rospy.loginfo(f"  is_simulation: {is_simulation}")
        rospy.loginfo(f"  target_classes: {target_classes}")
        rospy.loginfo(f"  conf_threshold: {conf_thres}")
        
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
    
    def _setup_ros_communication(self):
        """设置 ROS 订阅和发布"""
        # 获取话题名称
        ns = rospy.get_param('~param_namespace', 'object_detection')
        image_topic = rospy.get_param(f'/{ns}/topics/image_input', '/camera/color/image_raw')
        objects_topic = rospy.get_param(f'/{ns}/topics/detection_output', '/objects')
        control_topic = rospy.get_param(f'/{ns}/topics/control_input', '/detect_ctrl')
        
        # 订阅图像话题（事件驱动，收到图像即处理）
        self.image_sub = rospy.Subscriber(
            image_topic,
            Image,
            self._image_callback,
            queue_size=1  # 只保留最新的图像
        )
        
        # 订阅检测控制话题
        self.control_sub = rospy.Subscriber(
            control_topic,
            DetEnable,
            self._control_callback,
            queue_size=10
        )
        
        # 发布检测结果
        self.objects_pub = rospy.Publisher(
            objects_topic,
            Objects,
            queue_size=10
        )
        
        rospy.loginfo(f"Subscribed to: {image_topic}")
        rospy.loginfo(f"Publishing to: {objects_topic}")
    
    def _image_callback(self, msg: Image):
        """图像回调函数（事件驱动）
        
        收到图像消息后立即处理，而非固定频率轮询。
        
        Args:
            msg: ROS 图像消息
        """
        # 记录开始时间（用于计算检测耗时）
        import time
        start_time = time.time()

        self._cached_balloon_centers.clear()    
        
        # 打印首次接收日志
        if not self._first_image_received:
            rospy.loginfo(f"Received first image message! Size: {msg.width}x{msg.height}")
            self._first_image_received = True

        # 如果检测被禁用，直接返回
        if not self._detection_enabled:
            return
        
        # 转换图像格式
        try:
            # 根据仿真/真机环境进行不同的处理
            # 获取原始图像
            image_raw = self.bridge.imgmsg_to_cv2(msg, msg.encoding)
            
            # 统一转换为正确的 BGR 格式
            if self.config.is_simulation:
                # 仿真环境：原始图像即为正确的 BGR
                image_bgr = image_raw
            else:
                # 真机环境：原始图像通道顺序需要反转才能得到正确的 BGR
                # 这里的 ::-1 将图像从某种格式（可能是 RGB）转换为 BGR 以纠正颜色
                image_bgr = image_raw[:, :, ::-1]
            
            # 转换为 RGB 用于模型推理
            # 注意：YOLOE 模型通常需要 RGB 输入
            image_rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
                
        except Exception as e:
            rospy.logerr(f"Failed to convert image: {e}")
            return
        
        # 执行检测
        detections = self.detector.predict(image_rgb)
        
        # 如果有检测结果
        if detections is not None and len(detections) > 0:
            # 生成并发布 ROS 消息
            objects_msg = self._process_detections(
                detections, 
                image_bgr,
                msg.header
            )
            self.objects_pub.publish(objects_msg)
            
            rospy.logdebug(f"Detected {len(detections)} objects")
        
        # 计算处理时间（毫秒）
        processing_time_ms = (time.time() - start_time) * 1000.0
        
        # 可视化
        if self.show_visualization:
            self._visualize_detections(image_bgr, detections, processing_time_ms)
    
    def _control_callback(self, msg: DetEnable):
        """检测控制回调函数
        
        Args:
            msg: 检测控制消息
        """
        self._detection_enabled = msg.is_open
        rospy.loginfo(f"Detection {'enabled' if self._detection_enabled else 'disabled'}")
    
    def _classify_balloon_color(self, image: np.ndarray, bbox: tuple) -> str:
        """根据 HSV 颜色空间对气球进行颜色分类
        
        Args:
            image: BGR 格式图像
            bbox: 边界框 (x1, y1, x2, y2)
            
        Returns:
            颜色名称: 'red', 'blue', 或 'purple'
        """
        x1, y1, x2, y2 = bbox
        roi = image[y1:y2, x1:x2]
        if roi.size == 0:
            return "red"
        
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        
        # 红色 (H: 0-10 或 160-180, S > 50, V > 50)
        red_mask1 = cv2.inRange(hsv, np.array([0, 50, 50]), np.array([10, 255, 255]))
        red_mask2 = cv2.inRange(hsv, np.array([160, 50, 50]), np.array([180, 255, 255]))
        red_count = cv2.countNonZero(red_mask1) + cv2.countNonZero(red_mask2)
        
        # 蓝色 (H: 100-130, S > 50, V > 50)
        blue_mask = cv2.inRange(hsv, np.array([100, 50, 50]), np.array([130, 255, 255]))
        blue_count = cv2.countNonZero(blue_mask)
        
        # 紫色 (H: 130-160, S > 30, V > 30)
        purple_mask = cv2.inRange(hsv, np.array([130, 30, 30]), np.array([160, 255, 255]))
        purple_count = cv2.countNonZero(purple_mask)
        
        # 选择像素数最多的颜色
        counts = {"red": red_count, "blue": blue_count, "purple": purple_count}
        color = max(counts, key=counts.get)
        
        rospy.logdebug(f"Balloon color: {color} (R:{red_count} B:{blue_count} P:{purple_count})")
        return color
    
    def _process_detections(
        self, 
        detections: np.ndarray,
        image: np.ndarray,
        header
    ) -> Objects:
        """处理检测结果，生成 ROS 消息
        
        Args:
            detections: 检测结果数组 (N, 6): [x1, y1, x2, y2, conf, cls]
            image: 原始图像 (BGR 格式)
            header: ROS 消息头
            
        Returns:
            Objects 消息
        """
        objects_msg = Objects()
        objects_msg.header = header
        objects_msg.source_id = 0
        
        for det in detections:
            x1, y1, x2, y2, conf, cls_id = det
            cls_id = int(cls_id)
            
            # 过滤非目标类别
            if not self._is_target_class(cls_id):
                continue
            
            bbox = (int(x1), int(y1), int(x2), int(y2))
            
            # 创建对象消息
            obj = Obj()
            obj.class_name = self._get_class_name(cls_id)
            obj.left_top_x = bbox[0]
            obj.left_top_y = bbox[1]
            obj.right_bottom_x = bbox[2]
            obj.right_bottom_y = bbox[3]
            obj.score = float(conf)
            obj.header = header
            
            # 如果是气球（模型类名为 red_balloon），进行颜色分类 + 像素追踪
            if obj.class_name == "red_balloon":
                # HSV 颜色分类，重命名为实际颜色
                color = self._classify_balloon_color(image, bbox)
                obj.class_name = f"{color}_balloon"  # red_balloon / blue_balloon / purple_balloon
                
                center = self.balloon_tracker.track(image, bbox)
                if center is not None:
                    obj.center_x = center[0]
                    obj.center_y = center[1]
                else:
                    rospy.logwarn("Balloon pixel tracking failed, using bbox center")
                    obj.center_x = (bbox[0] + bbox[2]) // 2
                    obj.center_y = (bbox[1] + bbox[3]) // 2
                # ✅ 缓存气球中心点供可视化使用（避免重复计算）
                self._cached_balloon_centers[len(objects_msg.objects)] = (obj.center_x, obj.center_y)
            else:
                # 其他目标使用边界框中心
                obj.center_x = (bbox[0] + bbox[2]) // 2
                obj.center_y = (bbox[1] + bbox[3]) // 2
            
            objects_msg.objects.append(obj)
        
        return objects_msg
    
    def _visualize_detections(self, image: np.ndarray, detections: Optional[np.ndarray], processing_time_ms: float = 0.0):
        """可视化检测结果
        
        Args:
            image: 输入图像 (BGR 格式)
            detections: 检测结果
            processing_time_ms: 处理时间（毫秒）
        """
        img_vis = image.copy()
        
        if detections is not None and len(detections) > 0:
            # 绘制检测框
            for det in detections:
                x1, y1, x2, y2, conf, cls_id = det
                cls_id = int(cls_id)
                
                # 过滤非目标类别
                if not self._is_target_class(cls_id):
                    continue
                
                label = self._get_class_name(cls_id)
                
                img_vis = self.visualizer.draw_detection(
                    img_vis,
                    (int(x1), int(y1), int(x2), int(y2)),
                    label,
                    float(conf)
                )
                
                # 如果是气球，绘制追踪中心点
                if "balloon" in label:
                    balloon_idx = sum(1 for d in detections[:list(detections).index(det)] 
                      if self._is_target_class(int(d[5])) 
                      and "balloon" in self._get_class_name(int(d[5])))
                    center = self._cached_balloon_centers.get(balloon_idx)  
                    if center is not None:
                        img_vis = self.visualizer.draw_center_point(
                            img_vis, center
                        )
        # 转换为 RGB 用于显示（保持颜色正确）
        img_vis_rgb = cv2.cvtColor(img_vis, cv2.COLOR_BGR2RGB)
        
        try:
            # 第一次显示时创建可调整大小的窗口
            if not hasattr(self, '_window_created'):
                cv2.namedWindow('Object Detection', cv2.WINDOW_NORMAL)
                # 可选：设置初始窗口大小（宽度，高度），取消注释下面这行
                cv2.resizeWindow('Object Detection', 640, 480)
                self._window_created = True
                rospy.loginfo("==> Created resizable visualization window (WINDOW_NORMAL)")
            
            # 显示图像
            cv2.imshow('Object Detection', img_vis_rgb)
            cv2.waitKey(2)
            
            # 第一次成功显示时输出日志
            if not hasattr(self, '_first_display_logged'):
                self._first_display_logged = True
                rospy.loginfo("==> ✓ Image visualization is now displaying! You can resize the window by dragging its edges.")
                
        except Exception as e:
            # 每5秒最多输出一次错误，避免日志刷屏
            rospy.logwarn_throttle(5.0, f"Failed to display image: {e}")
        # 保存图像逻辑
        if self.config.save_images and detections is not None and len(detections) > 0:
            found_save_targets = []
            
            for det in detections:
                cls_id = int(det[5])
                # 只保存目标类别
                if self._is_target_class(cls_id):
                    label = self._get_class_name(cls_id)
                    # 检查是否在保存列表中
                    if label in self.config.save_classes:
                        if label not in found_save_targets:
                            found_save_targets.append(label)
            
            if found_save_targets:
                # 创建保存用的图像副本，在上面额外标注角点坐标
                img_to_save = img_vis.copy()
                
                # 在图像顶部显示检测时间
                time_text = f"Detection Time: {processing_time_ms:.1f} ms"
                font = cv2.FONT_HERSHEY_SIMPLEX
                font_scale = 0.6
                thickness = 2
                (text_w, text_h), baseline = cv2.getTextSize(time_text, font, font_scale, thickness)
                
                # 绘制半透明背景
                overlay = img_to_save.copy()
                cv2.rectangle(overlay, (5, 5), (text_w + 15, text_h + baseline + 15), (0, 0, 0), -1)
                cv2.addWeighted(overlay, 0.6, img_to_save, 0.4, 0, img_to_save)
                
                # 绘制时间文本
                cv2.putText(img_to_save, time_text, (10, text_h + 10),
                          font, font_scale, (0, 255, 0), thickness, cv2.LINE_AA)
                
                # 在检测框的关键点标注坐标
                for det in detections:
                    cls_id = int(det[5])
                    if self._is_target_class(cls_id):
                        label = self._get_class_name(cls_id)
                        if label in self.config.save_classes:
                            x1, y1, x2, y2 = int(det[0]), int(det[1]), int(det[2]), int(det[3])
                            
                            # 定义需要标注的关键点：左上角、右下角、中心点
                            center_x = (x1 + x2) // 2
                            center_y = (y1 + y2) // 2
                            
                            key_points = [
                                (x1, y1, "TL"),          # Top-Left 左上角
                                (x2, y2, "BR"),          # Bottom-Right 右下角
                                (center_x, center_y, "C")  # Center 中心点
                            ]
                            
                            # 计算面积占比
                            img_h, img_w = image.shape[:2]
                            bbox_area = (x2 - x1) * (y2 - y1)
                            img_area = img_w * img_h
                            area_ratio = bbox_area / img_area * 100  # 百分比
                            
                            # 在检测框下方显示面积占比
                            area_text = f"Area: {area_ratio:.1f}%"
                            font = cv2.FONT_HERSHEY_SIMPLEX
                            font_scale = 0.5
                            thickness = 1
                            (text_w, text_h), _ = cv2.getTextSize(area_text, font, font_scale, thickness)
                            
                            # 位置：检测框下方居中
                            area_text_x = center_x - text_w // 2
                            area_text_y = y2 + text_h + 8
                            
                            # 绘制背景
                            cv2.rectangle(img_to_save, 
                                        (area_text_x - 3, area_text_y - text_h - 3),
                                        (area_text_x + text_w + 3, area_text_y + 3),
                                        (0, 0, 0), -1)
                            
                            # 绘制面积文本（青色）
                            cv2.putText(img_to_save, area_text, (area_text_x, area_text_y),
                                      font, font_scale, (255, 255, 0), thickness, cv2.LINE_AA)
                            
                            # 在每个关键点标注坐标
                            for px, py, point_name in key_points:
                                coord_text = f"({px},{py})"
                                
                                # 绘制小圆点标记关键点
                                if point_name == "C":
                                    # 中心点用不同颜色（绿色）
                                    cv2.circle(img_to_save, (px, py), 5, (0, 255, 0), -1)
                                else:
                                    # 角点用黄色
                                    cv2.circle(img_to_save, (px, py), 4, (0, 255, 255), -1)
                                
                                # 计算文本位置（根据点的位置调整偏移）
                                font = cv2.FONT_HERSHEY_SIMPLEX
                                font_scale = 0.4
                                thickness = 1
                                (text_w, text_h), _ = cv2.getTextSize(coord_text, font, font_scale, thickness)
                                
                                # 根据点的位置调整文本偏移
                                if point_name == "TL":
                                    text_x, text_y = px + 6, py - 6
                                elif point_name == "BR":
                                    text_x, text_y = px - text_w - 6, py + text_h + 6
                                else:  # C (Center)
                                    text_x, text_y = px + 6, py + text_h + 6
                                
                                # 绘制文本背景
                                cv2.rectangle(img_to_save, 
                                            (text_x - 2, text_y - text_h - 2),
                                            (text_x + text_w + 2, text_y + 2),
                                            (0, 0, 0), -1)
                                
                                # 绘制坐标文本
                                cv2.putText(img_to_save, coord_text, (text_x, text_y),
                                          font, font_scale, (0, 255, 255), thickness, cv2.LINE_AA)
                
                # 生成文件名: timestamp_label1_label2.jpg
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
                labels_str = "_".join(found_save_targets)
                filename = f"{timestamp}_{labels_str}.jpg"
                filepath = os.path.join(self.save_dir, filename)
                
                try:
                    # 保存带有检测框和角点坐标的图像
                    cv2.imwrite(filepath, img_to_save)
                except Exception as e:
                    rospy.logwarn(f"Failed to save image: {e}")
    
    def run(self):
        """运行检测节点
        
        使用事件驱动模式，不需要主循环轮询。
        """
        rospy.loginfo("Object Detection Node is running (event-driven mode)...")
        rospy.loginfo("Waiting for images...")
        
        try:
            rospy.spin()  # 等待回调
        except KeyboardInterrupt:
            rospy.loginfo("Shutting down Object Detection Node...")
        finally:
            cv2.destroyAllWindows()


def main():
    """主函数"""
    try:
        # 创建并运行节点（配置从 ROS 参数服务器读取）
        node = ObjectDetectionNode()
        node.run()
        
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Fatal error: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()
