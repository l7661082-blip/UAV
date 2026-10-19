# 为什么 src 里面还有一个 object_det？

## 这是 ROS Python 包的标准结构！

在 ROS 中，Python 包的标准组织方式是：

```
your_ros_package/              # ROS 包根目录
├── CMakeLists.txt
├── package.xml
├── setup.py                   # 关键！定义 Python 包
├── scripts/                   # 可执行脚本（ROS 节点）
│   └── my_node.py
└── src/                       # Python 源代码目录
    └── your_ros_package/      # Python 包（与 ROS 包同名）
        ├── __init__.py
        └── my_module.py
```

## 为什么要这样设计？

### 1. **分离可执行脚本和可复用模块**

- `scripts/det.py`: **可执行的 ROS 节点**，用 `rosrun` 运行
- `src/object_det/`: **可复用的 Python 模块**，可以被其他代码导入

### 2. **符合 Python 包规范**

当你运行 `catkin_make` 时，ROS 会：
1. 读取 `setup.py`
2. 将 `src/object_det/` 安装为 Python 包
3. 这样你就可以在任何地方 `import object_det`

### 3. **实际例子**

**在 scripts/det.py 中**（可执行节点）：
```python
#!/usr/bin/env python
# 这是一个 ROS 节点，直接运行
import rospy
from object_det.detectors import YoloeDetector  # 导入我们的模块

detector = YoloeDetector(...)
# ... ROS 节点逻辑
```

**在 src/object_det/detectors/yoloe_detector.py 中**（可复用模块）：
```python
# 这是一个可复用的类，不直接运行
class YoloeDetector:
    def __init__(self, ...):
        # 检测器逻辑
```

## 目录对应关系

```
object_det/                    # ROS 包名
└── src/
    └── object_det/            # Python 包名（通常与 ROS 包同名）
        ├── __init__.py
        ├── detectors/         # 子模块
        │   ├── __init__.py
        │   └── yoloe_detector.py
        └── utils/
            └── __init__.py
```

**导入方式**：
```python
from object_det.detectors import YoloeDetector
from object_det.utils import some_function
```

## 这样做的好处

✅ **代码复用**：其他包可以导入你的模块  
✅ **清晰分离**：节点脚本 vs 库代码  
✅ **标准规范**：符合 ROS 和 Python 最佳实践  
✅ **易于测试**：可以单独测试模块，不需要运行整个 ROS 节点

## 总结

- `object_det/` = ROS 包根目录
- `object_det/src/object_det/` = Python 包（可导入的模块）
- 这**不是**重复，而是 ROS + Python 的标准结构！
