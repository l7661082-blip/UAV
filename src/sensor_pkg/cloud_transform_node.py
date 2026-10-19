#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
cloud_transform_node.py

将 RflySim 深度传感器发布的点云从传感器局部坐标系转换到 world (odom) 坐标系，
供 Ego-Planner 的 cloudCallback 直接使用。

订阅:
  - /rflysim/uav1/local/odom        (nav_msgs/Odometry)     真值里程计
  - /rflysim/sensor0/Depth_Cloud     (sensor_msgs/PointCloud2) 深度点云

发布:
  - /cloud_registered                (sensor_msgs/PointCloud2) world 坐标系下的点云
"""

import rospy
import numpy as np
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
import std_msgs.msg
from scipy.spatial.transform import Rotation


class CloudTransformNode:
    def __init__(self):
        rospy.init_node('cloud_transform_node', anonymous=True)

        # =====================================================================
        # 传感器外参配置 (来自 Config.json : SensorPosXYZ / SensorAngEular)
        # 深度传感器在机体坐标系下的位置偏移 [x, y, z] (m)
        # =====================================================================
        sensor_pos_x = rospy.get_param('~sensor_pos_x', 0.2)
        sensor_pos_y = rospy.get_param('~sensor_pos_y', 0.0)
        sensor_pos_z = rospy.get_param('~sensor_pos_z', 0.0)

        # 深度传感器在机体坐标系下的欧拉角 [roll, pitch, yaw] (度)
        sensor_ang_roll  = rospy.get_param('~sensor_ang_roll',  0.0)
        sensor_ang_pitch = rospy.get_param('~sensor_ang_pitch', 0.0)
        sensor_ang_yaw   = rospy.get_param('~sensor_ang_yaw',   0.0)

        # 话题名称 (可通过 rosparam 覆盖)
        odom_topic  = rospy.get_param('~odom_topic',  '/rflysim/uav1/local/odom')
        cloud_in    = rospy.get_param('~cloud_in',    '/rflysim/sensor0/Depth_Cloud')
        cloud_out   = rospy.get_param('~cloud_out',   '/cloud_registered')
        self.frame_id = rospy.get_param('~frame_id',  'world')

        # 是否已经是机体系点云 (True=传感器系, 需要先 sensor->body 再 body->world)
        # 如果 RflySim 发布的 Depth_Cloud 已经在机体系下，设为 False
        self.cloud_in_sensor_frame = rospy.get_param('~cloud_in_sensor_frame', True)

        # ---- 构建 sensor -> body 的齐次变换矩阵 T_body_sensor ----
        R_sensor = Rotation.from_euler(
            'xyz',
            [sensor_ang_roll, sensor_ang_pitch, sensor_ang_yaw],
            degrees=True
        ).as_matrix()

        self.T_body_sensor = np.eye(4)
        self.T_body_sensor[:3, :3] = R_sensor
        self.T_body_sensor[:3, 3]  = [sensor_pos_x, sensor_pos_y, sensor_pos_z]

        # ---- 状态变量 ----
        self.latest_odom = None
        self.odom_lock = False  # 简易锁 (单线程回调其实不需要，保险写上)

        # ---- 发布者 ----
        self.cloud_pub = rospy.Publisher(cloud_out, PointCloud2, queue_size=5)

        # ---- 订阅者 ----
        rospy.Subscriber(odom_topic, Odometry, self.odom_callback, queue_size=5)
        rospy.Subscriber(cloud_in, PointCloud2, self.cloud_callback, queue_size=5)

        rospy.loginfo("[CloudTransform] 节点已启动")
        rospy.loginfo("[CloudTransform]   odom  : %s", odom_topic)
        rospy.loginfo("[CloudTransform]   cloud in : %s", cloud_in)
        rospy.loginfo("[CloudTransform]   cloud out: %s", cloud_out)
        rospy.loginfo("[CloudTransform]   sensor offset: [%.3f, %.3f, %.3f]",
                      sensor_pos_x, sensor_pos_y, sensor_pos_z)
        rospy.loginfo("[CloudTransform]   sensor angles: [%.1f, %.1f, %.1f] deg",
                      sensor_ang_roll, sensor_ang_pitch, sensor_ang_yaw)
        rospy.loginfo("[CloudTransform]   cloud_in_sensor_frame: %s",
                      self.cloud_in_sensor_frame)

    # ---------------------------------------------------------------
    def odom_callback(self, msg):
        """缓存最新的里程计消息"""
        self.latest_odom = msg

    # ---------------------------------------------------------------
    def cloud_callback(self, msg):
        """将传感器坐标系下的点云转换到 world 坐标系并发布"""
        if self.latest_odom is None:
            rospy.logwarn_throttle(5.0, "[CloudTransform] 还未收到 odom 数据，跳过")
            return

        # ---------- 1. 从 odom 获取 body -> world 变换 -----------
        odom = self.latest_odom
        pos = odom.pose.pose.position
        ori = odom.pose.pose.orientation

        R_body = Rotation.from_quat([ori.x, ori.y, ori.z, ori.w]).as_matrix()
        T_world_body = np.eye(4)
        T_world_body[:3, :3] = R_body
        T_world_body[:3, 3]  = [pos.x, pos.y, pos.z]

        # ---------- 2. 计算完整变换 sensor -> world ---------------
        if self.cloud_in_sensor_frame:
            T_world_sensor = T_world_body @ self.T_body_sensor
        else:
            # 点云已经在机体系下，只需 body -> world
            T_world_sensor = T_world_body

        R_transform = T_world_sensor[:3, :3]
        t_transform = T_world_sensor[:3, 3]

        # ---------- 3. 读取原始点云 --------------------------------
        # 支持 xyz 和 xyzi 两种格式
        gen = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        points_list = list(gen)

        if len(points_list) == 0:
            return

        points = np.array(points_list, dtype=np.float32)  # (N, 3)

        # ---------- 4. 坐标变换 ------------------------------------
        # P_world = R * P_sensor + t
        points_world = (R_transform @ points.T).T + t_transform  # (N, 3)

        # ---------- 5. 组装并发布转换后的 PointCloud2 ---------------
        header = std_msgs.msg.Header()
        header.stamp = msg.header.stamp  # 保持原始时间戳
        header.frame_id = self.frame_id

        cloud_out = pc2.create_cloud_xyz32(header, points_world.tolist())
        self.cloud_pub.publish(cloud_out)

    # ---------------------------------------------------------------
    def spin(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        node = CloudTransformNode()
        node.spin()
    except rospy.ROSInterruptException:
        pass
