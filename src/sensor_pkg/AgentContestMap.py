from copy import deepcopy
from pickletools import pyset, string1
from re import A
import time
import math
from tracemalloc import start
import numpy
from scipy.spatial.transform import Rotation

import pygame
import threading
import UE4CtrlAPI as UE4CtrlAPI
import signal

# import cv2
import sys

# -*- coding: utf-8 -*-
import UE4CtrlAPI
import random
import os


ue = UE4CtrlAPI.UE4CtrlAPI()

class MapRandomizer:
    """
    Aruco:43    Aruco码
    H_small:910       起降点
    pillar_small:813  立柱
    Box:815     方框
    Gate:818     门洞
    Floor:819   地面
    Capter:820   地毯
    """
    def __init__(self):
        self.config = {
            813: {'range_x': (-0.1, 0.1), 'range_y': (-0.1, 0.1),   'name': 'pillar_small'}, 
            815: {'range_x': (-0.1, 0.1), 'range_y': (-0.1, 0.1),  'name': 'Box'},
            818: {'range_x': (-0.25, 0.25), 'range_y': (-0.05, 0.05),  'name': 'Gate'},
            43: {'range_x': (-0.1, 0.1), 'range_y': (-0.1, 0.1),  'name': 'Aruco'},
        }

    def apply_randomness(self, pos_list, ang_list):
        # 用于存储特定obj_id的共享随机值
        shared_dx = {}
        shared_dy = {}  # 存储813的dy基础值
        pillar_count = {}  # 追踪每种obj_id出现的次数
        distance_offset = random.uniform(-0.1, 0.1)  # 立柱间距离的随机变化
        
        for i, item in enumerate(pos_list):
            obj_id = item[0]           
            if obj_id in self.config:
                cfg = self.config[obj_id]
                current_yaw = ang_list[i][2]

                if obj_id == 818:  # Gate
                    if current_yaw > 0.5:
                        dx = random.uniform(cfg['range_x'][0], cfg['range_x'][1])
                        dy = random.uniform(cfg['range_y'][0], cfg['range_y'][1])
                    else:
                        dy = random.uniform(cfg['range_x'][0], cfg['range_x'][1])
                        dx = random.uniform(cfg['range_y'][0], cfg['range_y'][1])
                    item[1] += dx
                    item[2] += dy
                elif obj_id == 813:  # pillar_small
                    # 如果是第一次遇到813，生成并存储dx和dy值
                    if 813 not in shared_dx:
                        shared_dx[813] = random.uniform(cfg['range_x'][0], cfg['range_x'][1])
                        shared_dy[813] = random.uniform(cfg['range_y'][0], cfg['range_y'][1])
                        pillar_count[813] = 0
                    
                    dx = shared_dx[813]
                    # 第一个813使用基础dy，第二个813在基础dy上增加距离偏移
                    if pillar_count[813] == 0:
                        dy = shared_dy[813]
                    else:
                        dy = shared_dy[813] + distance_offset
                    
                    pillar_count[813] += 1
                    item[1] += dx
                    item[2] += dy
                else:
                    dx = random.uniform(cfg['range_x'][0], cfg['range_x'][1])
                    dy = random.uniform(cfg['range_y'][0], cfg['range_y'][1])

                    item[1] += dx
                    item[2] += dy


if __name__ == "__main__":

    # def move_up_directory(path, levels):
    #     for _ in range(levels):
    #         path = os.path.dirname(path)
    #     return path

    # # 获取当前脚本的目录
    # curPath = os.path.dirname(os.path.realpath(sys.argv[0]))

    # # 向上移动两层目录
    # newPath = move_up_directory(curPath, 2)

    # print("Current Path:", curPath)
    # print("New Path after moving up 2 levels:", newPath)

    psp_path = os.getenv("PSP_PATH")
    print(psp_path)  # 这将打印环境变量PSP_PATH的值，如 "C:\PX4PSP"
    #file_path = os.path.join(psp_path, "RflySim3D", "RflySim3D", "ChallengeMapID.txt")

    # 启动RflySim
    # ue.sendUE4Cmd(b'RflyChangeViewKeyCmd P')
    # time.sleep(0.5)
    # ue.sendUE4Cmd('r.setres 720x405w',0) # 设置UE4窗口分辨率，注意本窗口仅限于显示，取图分辨率在json中配置，本窗口设置越小，资源需求越少。
    # time.sleep(0.5)
    # ue.sendUE4Cmd('t.MaxFPS 20',0) # 设置UE4最大刷新频率，同时也是取图频率
    time.sleep(2)  # 切换场景需要一定加载时间

    """
    Aruco:100000043    Aruco码
    H_small:810       起降点
    pillar_small:813  立柱
    Car:814     小车
    Box:815     方框
    Gate:818     门洞
    Floor:819   地面
    Capter:820   tianhuaban
    """
    # x: 0-8  y: -2-2  范围
    scales = [
        [1, 1, 1],
        [3, 3, 1],
        [0.8, 0.8, 1],
        [0.5, 0.5, 1],
        [0.5, 0.5, 1],
        [1, 1, 1],
    ]
    pos = [
        [100100819, 4, 0, 0],
        [100100043, 4, -1, -0.02],
        [810, -0.1, -0.8, 0],
        [813, 1.2, -0.8, 0],
        [813, 1.2, 1, 0],
        [815, 3.25, 0.4, 0],
    ]
    ang = [
        [0, 0, 0],
        [0, math.radians(-90), 0],
        [0, 0, 0],
        [0, 0, 0],
        [0, 0, 0],
        [0, 0, math.radians(180)],
    ]

    randomizer = MapRandomizer()
    randomizer.apply_randomness(pos, ang)

    x = [0] * len(pos)
    y = [0] * len(pos)
    for i in range(len(pos)):
        ue.sendUE4PosScale(
            copterID=i + 10002,
            vehicleType=pos[i][0],
            PosE=pos[i][1:4],
            AngEuler=ang[i],
            Scale=scales[i]
        )

        x[i] = pos[i][1]
        y[i] = pos[i][2]
    print(x[2], y[2])
    
    # 设置三个小车的初始位置
    ue.sendUE4PosScale(copterID=2,vehicleType=825,PosE=[0,-2.1,0])
    ue.sendUE4PosScale(copterID=3,vehicleType=825,PosE=[5.6,-2.1,0])
    ue.sendUE4PosScale(copterID=4,vehicleType=825,PosE=[5,2.1,0])

    # sheng cheng tian hua ban
    ue.sendUE4PosScale(copterID=11,vehicleType=820,PosE=[0,0,-3])

    # 等待创建的小车初始化完成
    time.sleep(5)
    
    # 定义三种颜色：红、蓝、紫
    colors = [
        [1, 0, 0],  # 红色
        [0, 0, 1],  # 蓝色
        [1, 0, 1]   # 紫色
    ]
    
    # 随机打乱颜色顺序
    # random.shuffle(colors)
    
    # 为三个气球设置随机颜色
    ue.sendUE4ExtAct(2, [1, 1, 0.05] +colors[0] + [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
    ue.sendUE4ExtAct(3, [1, 1, 0.05] +colors[1] + [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
    ue.sendUE4ExtAct(4, [1, 1, 0.05] +colors[2] + [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ])  
    
    ue.sendUE4ExtAct(10002, [0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
    ue.sendUE4ExtAct(10003, [0.25, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
    
    # 定义三个小车的路径
    # 第一个小车: (0,-2.1,0) ↔ (5,-2.1,0) 直线往返
    path1 = [[0,-2.1], [5,-2.1], [0,-2.1]]
    # 第二个小车: (5.6,-2.1,0) ↔ (5.6,2.1,0) 直线往返
    path2 = [[5.6,-2.1], [5.6,2.1], [5.6,-2.1]]
    # 第三个小车: (5.6,2.1,0) ↔ (0,2.1,0) 直线往返
    path3 = [[5,2.1], [0,2.1], [5,2.1]]
    
    # 初始化三个小车的状态
    offset = 0.1  # 位置容差
    
    # 定义实际速度范围 (m/s)
    MIN_SPEED_MS = 0.05  # 最小速度 0.2 m/s
    MAX_SPEED_MS = 1  # 最大速度 1 m/s
    
    # 控制循环时间间隔
    LOOP_INTERVAL = 0.1  # 秒
    
    # 速度转换函数：实际速度(m/s) -> 每次循环的位移增量
    def speed_ms_to_step(speed_ms):
        return speed_ms * LOOP_INTERVAL
    
    # 为每个小车随机生成不同的初始实际速度(m/s)
    car1_speed_ms = random.uniform(MIN_SPEED_MS, MAX_SPEED_MS)
    car2_speed_ms = random.uniform(MIN_SPEED_MS, MAX_SPEED_MS)
    car3_speed_ms = random.uniform(MIN_SPEED_MS, MAX_SPEED_MS)
    
    # 转换为每次循环的位移增量
    car1_speed = speed_ms_to_step(car1_speed_ms)
    car2_speed = speed_ms_to_step(car2_speed_ms)
    car3_speed = speed_ms_to_step(car3_speed_ms)
    
    # 设置目标速度（初始时与当前速度相同）
    car1_target_speed = car1_speed
    car2_target_speed = car2_speed
    car3_target_speed = car3_speed
    
    # 速度平滑过渡系数（每次循环靠近目标速度的比例）
    speed_transition_rate = 0.02
    
    # 为每个小车随机生成速度变化时间间隔(秒)，范围在5到12秒之间
    car1_speed_interval = random.uniform(5, 12)
    car2_speed_interval = random.uniform(5, 12)
    car3_speed_interval = random.uniform(5, 12)
    
    # 记录上次速度变化的时间
    car1_last_speed_change = time.time()
    car2_last_speed_change = time.time()
    car3_last_speed_change = time.time()
    
    # 小车1的状态
    car1_pos = [0, -2.1, 0]
    car1_idx = 0
    car1_yaw = 0
    
    # 小车2的状态
    car2_pos = [5.6, -2.1, 0]
    car2_idx = 0
    car2_yaw = math.pi/2  # 初始朝向北
    
    # 小车3的状态
    car3_pos = [5, 2.1, 0]
    car3_idx = 0
    car3_yaw = math.pi  # 初始朝向西
    
    # 控制循环
    while True:
        current_time = time.time()
        
        # 检查是否需要改变小车1的目标速度
        if current_time - car1_last_speed_change >= car1_speed_interval:
            car1_speed_ms = random.uniform(MIN_SPEED_MS, MAX_SPEED_MS)
            car1_target_speed = speed_ms_to_step(car1_speed_ms)
            car1_speed_interval = random.uniform(5, 12)
            car1_last_speed_change = current_time
            # print(f"小车1目标速度变化: {car1_speed_ms:.3f} m/s, 下次变化间隔: {car1_speed_interval:.2f}秒")
        
        # 检查是否需要改变小车2的目标速度
        if current_time - car2_last_speed_change >= car2_speed_interval:
            car2_speed_ms = random.uniform(MIN_SPEED_MS, MAX_SPEED_MS)
            car2_target_speed = speed_ms_to_step(car2_speed_ms)
            car2_speed_interval = random.uniform(5, 12)
            car2_last_speed_change = current_time
            # print(f"小车2目标速度变化: {car2_speed_ms:.3f} m/s, 下次变化间隔: {car2_speed_interval:.2f}秒")
        
        # 检查是否需要改变小车3的目标速度
        if current_time - car3_last_speed_change >= car3_speed_interval:
            car3_speed_ms = random.uniform(MIN_SPEED_MS, MAX_SPEED_MS)
            car3_target_speed = speed_ms_to_step(car3_speed_ms)
            car3_speed_interval = random.uniform(5, 12)
            car3_last_speed_change = current_time
            # print(f"小车3目标速度变化: {car3_speed_ms:.3f} m/s, 下次变化间隔: {car3_speed_interval:.2f}秒")
        
        # 平滑过渡：让当前速度逐渐靠近目标速度
        car1_speed += (car1_target_speed - car1_speed) * speed_transition_rate
        car2_speed += (car2_target_speed - car2_speed) * speed_transition_rate
        car3_speed += (car3_target_speed - car3_speed) * speed_transition_rate
        
        # 控制第一个小车
        if car1_idx == len(path1) - 1:
            car1_idx = 0
        if abs(car1_pos[0] - path1[car1_idx][0]) < offset and abs(car1_pos[1] - path1[car1_idx][1]) < offset:
            dx = path1[car1_idx + 1][0] - path1[car1_idx][0]
            dy = path1[car1_idx + 1][1] - path1[car1_idx][1]
            car1_pos[0] = path1[car1_idx][0]
            car1_pos[1] = path1[car1_idx][1]
            car1_idx += 1
            car1_yaw = math.atan2(dy, dx)
        
        offset_x1 = math.cos(car1_yaw) * car1_speed
        offset_y1 = math.sin(car1_yaw) * car1_speed
        car1_pos[0] += offset_x1
        car1_pos[1] += offset_y1
        ue.sendUE4PosScale(copterID=2, vehicleType=825, PosE=car1_pos, AngEuler=[0, 0, car1_yaw])
        
        # 控制第二个小车
        if car2_idx == len(path2) - 1:
            car2_idx = 0
        if abs(car2_pos[0] - path2[car2_idx][0]) < offset and abs(car2_pos[1] - path2[car2_idx][1]) < offset:
            dx = path2[car2_idx + 1][0] - path2[car2_idx][0]
            dy = path2[car2_idx + 1][1] - path2[car2_idx][1]
            car2_pos[0] = path2[car2_idx][0]
            car2_pos[1] = path2[car2_idx][1]
            car2_idx += 1
            car2_yaw = math.atan2(dy, dx)
        
        offset_x2 = math.cos(car2_yaw) * car2_speed
        offset_y2 = math.sin(car2_yaw) * car2_speed
        car2_pos[0] += offset_x2
        car2_pos[1] += offset_y2
        ue.sendUE4PosScale(copterID=3, vehicleType=825, PosE=car2_pos, AngEuler=[0, 0, car2_yaw])
        
        # 控制第三个小车
        if car3_idx == len(path3) - 1:
            car3_idx = 0
        if abs(car3_pos[0] - path3[car3_idx][0]) < offset and abs(car3_pos[1] - path3[car3_idx][1]) < offset:
            dx = path3[car3_idx + 1][0] - path3[car3_idx][0]
            dy = path3[car3_idx + 1][1] - path3[car3_idx][1]
            car3_pos[0] = path3[car3_idx][0]
            car3_pos[1] = path3[car3_idx][1]
            car3_idx += 1
            car3_yaw = math.atan2(dy, dx)
        
        offset_x3 = math.cos(car3_yaw) * car3_speed
        offset_y3 = math.sin(car3_yaw) * car3_speed
        car3_pos[0] += offset_x3
        car3_pos[1] += offset_y3
        ue.sendUE4PosScale(copterID=4, vehicleType=825, PosE=car3_pos, AngEuler=[0, 0, car3_yaw])
        
        time.sleep(LOOP_INTERVAL)
