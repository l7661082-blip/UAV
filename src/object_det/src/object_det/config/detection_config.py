#!/usr/bin/env python
# -*- coding: UTF-8 -*-
"""检测配置类

提供目标检测的所有配置参数。
"""

import os
from typing import List, Tuple


class DetectionConfig:
    """目标检测配置类
    
    集中管理所有检测相关的配置参数，包括模型路径、检测阈值、
    目标类别等。使用配置类可以方便地管理和修改参数。
    
    Attributes:
        model_path: YOLOE 模型文件路径
        conf_thres: 置信度阈值，范围 [0, 1]
        use_half: 是否使用半精度（FP16）推理
        max_det: 单张图像最大检测数量
        target_classes: 需要检测的目标类别列表
        red_channel_threshold: 气球红色通道阈值
    """
    
    def __init__(
        self,
        model_path: str = None,
        conf_thres: float = 0.5,
        use_half: bool = False,
        max_det: int = 50,
        target_classes: List[str] = None,
        red_channel_threshold: int = 60,
        is_simulation: bool = True,
        save_images: bool = False,
        save_classes: List[str] = None
    ):
        """初始化配置
        
        Args:
            model_path: 模型文件路径，如果为 None 则使用默认路径
            conf_thres: 置信度阈值
            use_half: 是否使用半精度推理
            max_det: 最大检测数量
            target_classes: 目标类别列表，默认为 ["frame", "balloon"]
            red_channel_threshold: 气球红色通道阈值
            is_simulation: 是否为仿真环境
        """
        # 模型路径
        if model_path is None:
            # 自动推断默认路径
            script_dir = os.path.dirname(os.path.abspath(__file__))
            pkg_dir = os.path.dirname(os.path.dirname(os.path.dirname(script_dir)))
            model_path = os.path.join(pkg_dir, 'models', 'best.pt')
        self.model_path = model_path
        
        # 检测参数
        self.conf_thres = conf_thres
        self.use_half = use_half
        self.max_det = max_det
        
        # 目标类别
        if target_classes is None:
            target_classes = ["frame", "balloon"]
        self.target_classes = target_classes
        
        # 气球追踪参数
        self.red_channel_threshold = red_channel_threshold
        
        # 环境参数
        self.is_simulation = is_simulation

        # 图像保存参数
        self.save_images = save_images if save_images is not None else False
        self.save_classes = save_classes if save_classes is not None else []
    
    def __repr__(self) -> str:
        """字符串表示"""
        return (
            f"DetectionConfig(\n"
            f"  model_path='{self.model_path}',\n"
            f"  conf_thres={self.conf_thres},\n"
            f"  target_classes={self.target_classes},\n"
            f"  is_simulation={self.is_simulation},\n"
            f"  red_threshold={self.red_channel_threshold},\n"
            f"  save_images={self.save_images},\n"
            f"  save_classes={self.save_classes}\n"
            f")"
        )
