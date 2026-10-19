#!/usr/bin/env python
# -*- coding: UTF-8 -*-
"""可视化工具

提供检测结果的可视化功能。
"""

import cv2
import numpy as np
from typing import List, Tuple, Optional


class DetectionVisualizer:
    """检测结果可视化器
    
    用于在图像上绘制检测框、标签等可视化信息。
    
    Attributes:
        font: OpenCV 字体
        font_scale: 字体大小
        thickness: 线条粗细
        box_color: 边界框颜色 (B, G, R)
        text_color: 文本颜色 (B, G, R)
    """
    
    def __init__(
        self,
        font: int = cv2.FONT_HERSHEY_SIMPLEX,
        font_scale: float = 0.6,
        thickness: int = 2,
        box_color: Tuple[int, int, int] = (0, 255, 0),
        text_color: Tuple[int, int, int] = (0, 255, 0)
    ):
        """初始化可视化器
        
        Args:
            font: OpenCV 字体类型
            font_scale: 字体缩放比例
            thickness: 线条粗细
            box_color: 边界框颜色
            text_color: 文本颜色
        """
        self.font = font
        self.font_scale = font_scale
        self.thickness = thickness
        self.box_color = box_color
        self.text_color = text_color
    
    def draw_detection(
        self,
        image: np.ndarray,
        bbox: Tuple[int, int, int, int],
        label: str,
        confidence: float
    ) -> np.ndarray:
        """绘制单个检测结果
        
        Args:
            image: 输入图像
            bbox: 边界框 (x1, y1, x2, y2)
            label: 类别标签
            confidence: 置信度
            
        Returns:
            绘制了检测结果的图像
        """
        img_vis = image.copy()
        x1, y1, x2, y2 = bbox
        
        # 绘制边界框
        cv2.rectangle(
            img_vis, 
            (int(x1), int(y1)), 
            (int(x2), int(y2)), 
            self.box_color, 
            self.thickness
        )
        
        # 绘制标签
        text = f"{label} {confidence:.2f}"
        text_y = max(20, int(y1) - 8)
        cv2.putText(
            img_vis, 
            text, 
            (int(x1), text_y),
            self.font, 
            self.font_scale, 
            self.text_color, 
            self.thickness
        )
        
        return img_vis
    
    def draw_detections(
        self,
        image: np.ndarray,
        detections: np.ndarray,
        class_names: List[str]
    ) -> np.ndarray:
        """绘制多个检测结果
        
        Args:
            image: 输入图像
            detections: 检测结果数组 (N, 6): [x1, y1, x2, y2, conf, cls]
            class_names: 类别名称列表
            
        Returns:
            绘制了所有检测结果的图像
        """
        img_vis = image.copy()
        
        if detections is None or len(detections) == 0:
            return img_vis
        
        for det in detections:
            x1, y1, x2, y2, conf, cls_id = det
            cls_id = int(cls_id)
            
            # 获取类别名称
            if 0 <= cls_id < len(class_names):
                label = class_names[cls_id]
            else:
                label = f"class_{cls_id}"
            
            # 绘制检测结果
            img_vis = self.draw_detection(
                img_vis,
                (int(x1), int(y1), int(x2), int(y2)),
                label,
                float(conf)
            )
        
        return img_vis
    
    def draw_center_point(
        self,
        image: np.ndarray,
        center: Tuple[int, int],
        color: Tuple[int, int, int] = (0, 255, 0),
        radius: int = 5
    ) -> np.ndarray:
        """绘制中心点
        
        Args:
            image: 输入图像
            center: 中心点坐标 (x, y)
            color: 点的颜色
            radius: 点的半径
            
        Returns:
            绘制了中心点的图像
        """
        img_vis = image.copy()
        cv2.circle(img_vis, center, radius, color, -1)
        return img_vis
