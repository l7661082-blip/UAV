#!/usr/bin/env python
# -*- coding: UTF-8 -*-
"""气球追踪器

通过红色像素追踪获得气球的精确中心点位置。
"""

import cv2
import numpy as np
from typing import Optional, Tuple


class BalloonTracker:
    """气球红色像素追踪器
    
    该追踪器用于在检测到的气球边界框内，通过分析红色通道
    找到红色像素的中心点，从而获得比边界框中心更精确的位置。
    
    Attributes:
        red_threshold: 红色通道阈值，像素值大于此阈值才被认为是红色
    """
    
    def __init__(self, red_threshold: int = 60):
        """初始化追踪器
        
        Args:
            red_threshold: 红色通道阈值，默认 60
        """
        self.red_threshold = red_threshold
    
    def track(
        self, 
        image: np.ndarray, 
        bbox: Tuple[int, int, int, int]
    ) -> Optional[Tuple[int, int]]:
        """追踪气球中心点
        
        在给定的边界框内寻找红色像素，并计算其中心点。
        
        Args:
            image: 输入图像，BGR 格式
            bbox: 边界框 (x1, y1, x2, y2)
            
        Returns:
            红色像素中心点坐标 (x, y)，如果没有找到红色像素则返回 None
            
        Example:
            >>> tracker = BalloonTracker(red_threshold=60)
            >>> center = tracker.track(image, (100, 100, 200, 200))
            >>> if center:
            >>>     print(f"Balloon center at: {center}")
        """
        x1, y1, x2, y2 = bbox
        
        # 提取 ROI（感兴趣区域）
        roi = image[y1:y2, x1:x2]
        
        # 提取红色通道
        red_channel = roi[:, :, 2]
        
        # 创建红色掩码
        red_mask = red_channel > self.red_threshold
        
        # 获取红色像素坐标
        red_pixel_coords = np.column_stack(np.where(red_mask))
        
        # 如果没有找到红色像素
        if len(red_pixel_coords) == 0:
            return None
        
        # 计算红色像素的中心位置（相对于 ROI）
        center_roi = np.mean(red_pixel_coords, axis=0).astype(int)
        
        # 转换为原始图像坐标系
        # 注意：np.where 返回的是 (row, col)，即 (y, x)
        center_x = center_roi[1] + x1
        center_y = center_roi[0] + y1
        
        return (center_x, center_y)
    
    def visualize(
        self, 
        image: np.ndarray, 
        bbox: Tuple[int, int, int, int],
        center: Optional[Tuple[int, int]] = None
    ) -> np.ndarray:
        """可视化追踪结果
        
        在图像上绘制边界框和红色像素中心点。
        
        Args:
            image: 输入图像
            bbox: 边界框
            center: 中心点坐标，如果为 None 则自动追踪
            
        Returns:
            绘制了可视化信息的图像
        """
        img_vis = image.copy()
        x1, y1, x2, y2 = bbox
        
        # 如果没有提供中心点，则自动追踪
        if center is None:
            center = self.track(image, bbox)
        
        # 提取 ROI 并创建红色掩码
        roi = image[y1:y2, x1:x2]
        red_channel = roi[:, :, 2]
        red_mask = red_channel > self.red_threshold
        
        # 在 ROI 区域标记红色像素
        img_vis[y1:y2, x1:x2][red_mask] = [0, 0, 255]
        
        # 绘制中心点
        if center is not None:
            cv2.circle(img_vis, center, 5, (0, 255, 0), -1)
        
        return img_vis
