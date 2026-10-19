#!/usr/bin/env python
# -*- coding: UTF-8 -*-
"""YOLOE 检测器封装类

提供统一的接口来使用 YOLOE 模型进行目标检测。
"""

import sys
import os
import torch
import numpy as np
import cv2

# CRITICAL FIX: YOLOE dynamic prompt embedding modifications conflict with PyTorch inference_mode
# resulting in "RuntimeError: Inference tensors do not track version counter".
# We force PyTorch to use no_grad() instead of inference_mode() across the board.
if hasattr(torch, 'inference_mode'):
    torch.inference_mode = torch.no_grad

# 确保能找到 third_party 中的 ultralytics
pkg_path = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
third_party_path = os.path.join(pkg_path, 'third_party')
if third_party_path not in sys.path:
    sys.path.insert(0, third_party_path)

from ultralytics import YOLOE


class YoloeDetector:
    """YOLOE 检测器封装类
    
    提供简单易用的接口来使用 YOLOE 模型进行目标检测。
    
    Args:
        model_path: 模型文件路径
        conf_thres: 置信度阈值，默认 0.5
        device: 设备，'cuda:0' 或 'cpu'，默认自动检测
        class_names: 类别名称列表
        use_half: 是否使用半精度（FP16），默认 False
        max_det: 最大检测数量，默认 50
    """
    
    def __init__(self, model_path, conf_thres=0.5, device=None, 
                 class_names=None, use_half=False, max_det=50):
        # 设备检测
        if device is None:
            self.device = "cuda:0" if torch.cuda.is_available() else "cpu"
        else:
            self.device = device
            
        # CUDA 优化
        if self.device.startswith("cuda"):
            torch.backends.cudnn.enabled = True
            torch.backends.cudnn.benchmark = True
            torch.backends.cudnn.deterministic = False
            torch.backends.cuda.matmul.allow_tf32 = True
        
        self.conf_thres = conf_thres
        self.use_half = use_half
        self.max_det = max_det
        self.class_names = class_names or []
        
        # 加载模型
        print(f"Loading YOLOE model: {model_path}")
        self.model = YOLOE(model_path)
        self.model.to(self.device)
        self.model.eval()
        
        # 设置类别名称
        if self.class_names:
            try:
                text_pe = self.model.get_text_pe(self.class_names)
                self.model.set_classes(self.class_names, text_pe)
                print(f"set_classes OK, classes={len(self.class_names)}")
            except Exception as e:
                print(f"Warning: set_classes failed: {e}")
        
        print(f"Device={self.device}, half={use_half}, conf={conf_thres}")

    @property
    def names(self):
        """获取模型的类别字典 {id: name}"""
        if hasattr(self.model, 'names'):
            return self.model.names
        elif hasattr(self.model.model, 'names'):
            return self.model.model.names
        return {}
    
    def predict(self, image, conf=None, max_det=None, return_vis=False):
        """执行目标检测
        
        Args:
            image: 输入图像 (BGR 格式的 numpy array)
            conf: 置信度阈值，默认使用初始化时的值
            max_det: 最大检测数量，默认使用初始化时的值
            return_vis: 是否返回可视化图像
        
        Returns:
            det: numpy array (N, 6) [x1, y1, x2, y2, conf, cls]
            img_vis: 可视化图像 (仅当 return_vis=True 时返回)
        """
        conf = conf or self.conf_thres
        max_det = max_det or self.max_det
        
        # 执行推理
        results = self.model.predict(
            source=image,
            conf=conf,
            device=self.device,
            half=(self.use_half and self.device.startswith("cuda")),
            verbose=False,
            max_det=max_det
        )
        
        # 解析结果
        det = None
        img_vis = image.copy() if return_vis else None
        
        if results and results[0].boxes is not None and len(results[0].boxes) > 0:
            b = results[0].boxes
            xyxy = b.xyxy.detach().cpu().numpy()  # (N, 4)
            conf_arr = b.conf.detach().cpu().numpy()  # (N,)
            cls = b.cls.detach().cpu().numpy().astype(int)  # (N,)
            
            # 组合成 (N, 6) 格式: [x1, y1, x2, y2, conf, cls]
            det = np.column_stack([xyxy, conf_arr, cls])
            
            # 绘制可视化
            if return_vis:
                for (x1, y1, x2, y2), c, cls_id in zip(xyxy, conf_arr, cls):
                    # 获取类别名称
                    if 0 <= cls_id < len(self.class_names):
                        name = self.class_names[cls_id]
                    else:
                        name = str(cls_id)
                    
                    label = f"{name} {c:.2f}"
                    cv2.rectangle(img_vis, (int(x1), int(y1)), (int(x2), int(y2)), 
                                (0, 255, 0), 2)
                    ty = max(20, int(y1) - 8)
                    cv2.putText(img_vis, label, (int(x1), ty),
                              cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        if return_vis:
            return det, img_vis
        return det
    
    def __call__(self, image):
        """使检测器可以像函数一样调用
        
        Args:
            image: 输入图像
            
        Returns:
            det: 检测结果
        """
        return self.predict(image)
