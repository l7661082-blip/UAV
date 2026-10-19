"""object_det - ROS 目标检测包

这个包提供基于 YOLOE 的目标检测功能。
"""

__version__ = "1.0.0"

# 导出主要类
from .config import DetectionConfig
from .detectors import YoloeDetector
from .trackers import BalloonTracker
from .utils import DetectionVisualizer

__all__ = [
    'DetectionConfig',
    'YoloeDetector',
    'BalloonTracker',
    'DetectionVisualizer'
]
