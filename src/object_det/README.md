# object_det 包目录结构说明

## 新的目录结构

```
object_det/
├── CMakeLists.txt          # ROS 构建配置
├── package.xml             # ROS 包信息
├── setup.py                # Python 包安装配置
│
├── launch/                 # ROS launch 文件
│   └── detection.launch
│
├── config/                 # 配置文件目录（预留）
│
├── models/                 # 模型权重文件
│   ├── frame_balloon_V4.pt
│   └── best.pt
│
├── scripts/                # ROS 节点脚本（可执行）
│   ├── det.py             # 检测节点
│   ├── auto_pose_ros1.py  # 自动姿态节点
│   └── ObjectDetect.py    # 旧的 YOLO 检测器（保留）
│
├── src/object_det/         # Python 源代码模块
│   ├── __init__.py
│   ├── detectors/         # 检测器模块
│   │   ├── __init__.py
│   │   └── yoloe_detector.py     # YOLOE 检测器封装
│   ├── utils/             # 工具函数
│   │   └── __init__.py
│   └── core/              # 核心功能
│       └── __init__.py
│
└── third_party/           # 第三方依赖
    ├── ultralytics/       # YOLOE 版本的 ultralytics
    ├── CLIP/
    ├── sam2/
    └── ...
```

## 主要改进

### 1. 清晰的职责分离
- `scripts/`: 只放可执行的 ROS 节点脚本
- `src/object_det/`: Python 模块代码（可复用的组件）
- `models/`: 模型权重文件
- `third_party/`: 第三方依赖库
- `config/`: 配置文件（YAML 等）

### 2. 模型路径更新
原来：`scripts/Model/frame_balloon_V4.pt`  
现在：`models/frame_balloon_V4.pt`

在代码中使用：
```python
pkg_dir = os.path.dirname(script_dir)
Model_path = os.path.join(pkg_dir, 'models', 'frame_balloon_V4.pt')
```

### 3. YOLOE 导入修复
在 `det.py` 中，通过以下方式确保能找到本地的 ultralytics：
```python
script_path = os.path.abspath(__file__)
script_dir = os.path.dirname(script_path)
if script_dir not in sys.path:
    sys.path.insert(0, script_dir)

from ultralytics import YOLOE
```

### 4. 安装依赖

需要安装以下 Python 库：

```bash
pip3 install -r requirements.txt
# 或者手动安装
pip3 install ultralytics ftfy regex
```

注意：`ftfy` 和 `regex` 库是 CLIP 文本处理所必需的，如果缺少会导致 set_classes 失败。

### 5. Python 包结构
创建了标准的 Python 包 `object_det`，可以在其他地方导入：
```python
from object_det.detectors import YoloeDetector

detector = YoloeDetector(
    model_path='/path/to/model.pt',
    class_names=['frame', 'balloon'],
    conf_thres=0.5
)
det = detector.predict(image)
```

## 使用方法

### 运行检测节点
```bash
cd ~/28com_ws
catkin_make
source devel/setup.bash
rosrun object_det det.py
```

### 查看检测结果
```bash
rostopic echo /objects
```

## 文件说明

### scripts/det.py
主要的检测节点，使用 YOLOE 模型进行目标检测。
- 订阅：`/camera/color/image_raw`
- 发布：`/objects`
- 功能：检测 frame 和 balloon，对 balloon 进行红色像素追踪

### src/object_det/detectors/yoloe_detector.py
YOLOE 检测器的封装类，提供简单易用的接口。
可以在其他脚本中复用。

### third_party/
包含项目依赖的第三方库：
- `ultralytics/`: 支持 YOLOE 的 ultralytics 库
- `CLIP/`, `sam2/` 等：其他依赖

## 注意事项

1. **third_party 路径**: scripts 中的脚本会自动将 scripts 目录添加到 sys.path，这样可以找到 third_party 中的库

2. **模型文件**: 确保模型文件在 `models/` 目录下

3. **编译**: 修改 CMakeLists.txt 后需要重新 catkin_make
