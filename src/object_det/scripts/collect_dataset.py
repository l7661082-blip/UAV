#!/usr/bin/env python
# -*- coding: UTF-8 -*-
"""数据集采集工具

订阅相机话题，按回车保存图片到数据集目录。

使用方法:
    rosrun object_det collect_dataset.py
    
    # 或者指定保存目录和类别名
    rosrun object_det collect_dataset.py _save_dir:=/path/to/dataset _class_name:=balloon

操作:
    - 按 Enter 保存当前帧
    - 按 q + Enter 退出
    - 按 s + Enter 切换类别

Author: Object Detection Team
Date: 2026-01-21
"""

import os
import sys
import cv2
import rospy
import threading
from datetime import datetime
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class DatasetCollector:
    """数据集采集器
    
    订阅相机话题，按回车保存图片。
    """
    
    def __init__(self):
        # 初始化 ROS 节点
        rospy.init_node('dataset_collector', anonymous=True)
        
        # 参数
        self.save_dir = rospy.get_param('~save_dir', os.path.expanduser('../dataset'))
        self.class_name = rospy.get_param('~class_name', 'images')
        self.image_topic = rospy.get_param('~image_topic', '/camera/color/image_raw')
        self.is_simulation = rospy.get_param('~is_simulation', False)
        
        # 创建保存目录
        self.current_save_dir = os.path.join(self.save_dir, self.class_name)
        os.makedirs(self.current_save_dir, exist_ok=True)
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # 当前图像
        self.current_image = None
        self.image_lock = threading.Lock()
        
        # 统计
        self.saved_count = 0
        self.total_by_class = {}
        
        # 订阅图像
        self.image_sub = rospy.Subscriber(
            self.image_topic,
            Image,
            self._image_callback,
            queue_size=1
        )
        
        # 打印信息
        self._print_banner()
    
    def _print_banner(self):
        """打印使用说明"""
        print("\n" + "=" * 60)
        print("       数据集采集工具 - Dataset Collector")
        print("=" * 60)
        print(f"  图像话题: {self.image_topic}")
        print(f"  保存目录: {self.save_dir}")
        print(f"  当前类别: {self.class_name}")
        print(f"  仿真模式: {self.is_simulation}")
        print("-" * 60)
        print("  操作说明:")
        print("    [Enter]      - 保存当前帧")
        print("    [s + Enter]  - 切换/新建类别")
        print("    [l + Enter]  - 列出已采集数量")
        print("    [q + Enter]  - 退出")
        print("=" * 60 + "\n")
    
    def _image_callback(self, msg):
        """图像回调"""
        try:
            # 转换图像
            image = self.bridge.imgmsg_to_cv2(msg, msg.encoding)
            
            # 真机模式需要转换颜色通道
            if not self.is_simulation:
                image = image[:, :, ::-1]
            
            with self.image_lock:
                self.current_image = image.copy()
                
        except Exception as e:
            rospy.logerr(f"Image conversion error: {e}")
    
    def _generate_filename(self):
        """生成文件名"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
        return f"{self.class_name}_{timestamp}.jpg"
    
    def save_current_image(self):
        """保存当前图像"""
        with self.image_lock:
            if self.current_image is None:
                print("  ⚠ 没有接收到图像，请检查话题是否正确发布")
                return False
            
            image_to_save = self.current_image.copy()
        
        # 生成文件名和路径
        filename = self._generate_filename()
        filepath = os.path.join(self.current_save_dir, filename)
        
        # 保存图像
        cv2.imwrite(filepath, image_to_save)
        
        # 更新统计
        self.saved_count += 1
        if self.class_name not in self.total_by_class:
            self.total_by_class[self.class_name] = 0
        self.total_by_class[self.class_name] += 1
        
        print(f"  ✓ 已保存 [{self.class_name}]: {filename}  (总计: {self.total_by_class[self.class_name]})")
        return True
    
    def switch_class(self):
        """切换类别"""
        print("\n  当前已有类别:", list(self.total_by_class.keys()) if self.total_by_class else "无")
        new_class = input("  输入新类别名称 (直接回车取消): ").strip()
        
        if new_class:
            self.class_name = new_class
            self.current_save_dir = os.path.join(self.save_dir, self.class_name)
            os.makedirs(self.current_save_dir, exist_ok=True)
            print(f"  ✓ 已切换到类别: {self.class_name}")
            print(f"    保存目录: {self.current_save_dir}")
        else:
            print("  取消切换")
        print()
    
    def list_stats(self):
        """列出统计信息"""
        print("\n  已采集数量统计:")
        print("  " + "-" * 30)
        total = 0
        for cls, count in self.total_by_class.items():
            print(f"    {cls}: {count} 张")
            total += count
        print("  " + "-" * 30)
        print(f"    总计: {total} 张\n")
    
    def show_preview(self):
        """显示预览窗口"""
        window_name = "Dataset Collector Preview (按窗口中的键也可操作)"
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window_name, 640, 480)
        
        while not rospy.is_shutdown():
            with self.image_lock:
                if self.current_image is not None:
                    display_img = self.current_image.copy()
                else:
                    display_img = None
            
            if display_img is not None:
                # 添加信息文字
                info_text = f"Class: {self.class_name} | Saved: {self.total_by_class.get(self.class_name, 0)}"
                cv2.putText(display_img, info_text, (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                cv2.putText(display_img, "Press SPACE to save, Q to quit", (10, 60), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
                
                cv2.imshow(window_name, display_img)
            
            key = cv2.waitKey(30) & 0xFF
            if key == ord(' '):  # 空格保存
                self.save_current_image()
            elif key == ord('q'):  # Q 退出
                break
            elif key == ord('s'):  # S 切换类别
                self.switch_class()
        
        cv2.destroyAllWindows()
    
    def run(self):
        """运行采集器（命令行模式）"""
        print("等待图像...")
        
        # 等待第一帧图像
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            with self.image_lock:
                if self.current_image is not None:
                    break
            rate.sleep()
        
        print("✓ 已接收到图像，开始采集！\n")
        print(f"当前类别: {self.class_name}")
        print("按 Enter 保存图片，输入 q 退出，输入 s 切换类别\n")
        
        # 启动预览线程
        preview_thread = threading.Thread(target=self.show_preview, daemon=True)
        preview_thread.start()
        
        # 主循环：等待用户输入
        while not rospy.is_shutdown():
            try:
                user_input = input(f"[{self.class_name}] >>> ").strip().lower()
                
                if user_input == '':
                    # 回车：保存图片
                    self.save_current_image()
                elif user_input == 'q':
                    # 退出
                    print("\n采集结束！")
                    self.list_stats()
                    break
                elif user_input == 's':
                    # 切换类别
                    self.switch_class()
                elif user_input == 'l':
                    # 列出统计
                    self.list_stats()
                else:
                    print(f"  未知命令: {user_input}")
                    print("  可用命令: [Enter]=保存, s=切换类别, l=统计, q=退出")
                    
            except EOFError:
                break
            except KeyboardInterrupt:
                print("\n\n采集中断！")
                self.list_stats()
                break


def main():
    try:
        collector = DatasetCollector()
        collector.run()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
