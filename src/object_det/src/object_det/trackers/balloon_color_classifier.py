#!/usr/bin/env python
# -*- coding: UTF-8 -*-
"""气球颜色分类器

在 YOLO 检测框内，通过 HSV 颜色分析区分红/蓝/紫气球。
"""

import cv2
import numpy as np
from typing import Optional, Tuple


class BalloonColorClassifier:
    """基于 HSV 的气球颜色分类器
    
    在 YOLO 检测到 balloon 后，对检测框 ROI 进行 HSV 颜色分析，
    判断是 red / blue / purple 气球，并返回该颜色区域的质心坐标。
    """

    # HSV 颜色阈值（与 hsv_det.py 保持一致）
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

    # 最少像素数，低于此值认为该颜色不存在
    MIN_PIXEL_COUNT = 30

    def classify(
        self, 
        image_bgr: np.ndarray, 
        bbox: Tuple[int, int, int, int]
    ) -> Tuple[str, Optional[Tuple[int, int]]]:
        """对检测框内的气球进行颜色分类
        
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
        
        roi_hsv = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2HSV)

        best_color = "unknown"
        best_count = 0
        best_center = None

        for color_name, ranges in self.COLOR_RANGES.items():
            # 合并多个 HSV 区间
            mask = np.zeros(roi_hsv.shape[:2], dtype=np.uint8)
            for r in ranges:
                mask |= cv2.inRange(roi_hsv, r["lower"], r["upper"])
            
            count = int(np.count_nonzero(mask))
            
            if count > best_count and count >= self.MIN_PIXEL_COUNT:
                best_count = count
                best_color = color_name

                # 计算质心
                coords = np.column_stack(np.where(mask > 0))
                if len(coords) > 0:
                    cy_roi, cx_roi = np.mean(coords, axis=0).astype(int)
                    best_center = (cx_roi + x1, cy_roi + y1)

        return best_color, best_center
