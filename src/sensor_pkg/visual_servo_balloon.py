#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
visual_servo_balloon.py

视觉伺服刺破气球节点 —— 基于 body 系 VX + VZ + yaw_rate 控制

策略：
  1. 对齐阶段 (ALIGN)  ：原地旋转 yaw_rate，使目标气球水平居中
  2. 逼近阶段 (APPROACH)：保持 yaw_rate 修正的同时，以 body VX 前进 + VZ 高度修正
  3. 冲刺阶段 (SPRINT)  ：当气球面积占比达到阈值，全速冲刺一段固定时间
  4. 完成 (DONE) / 搜索 (SEARCH)：丢失目标后原地慢转搜索

订阅:
  - /objects               (common_msgs/Objects)       HSV 检测结果
  - /rflysim/uav1/local/odom  (Odometry)              里程计

发布:
  - /mavros/setpoint_raw/local  (mavros_msgs/PositionTarget)  body 系速度指令
"""

import rospy
import numpy as np
from common_msgs.msg import Objects
from mavros_msgs.msg import PositionTarget
from nav_msgs.msg import Odometry


class State:
    SEARCH   = "SEARCH"
    ALIGN    = "ALIGN"
    APPROACH = "APPROACH"
    SPRINT   = "SPRINT"
    DONE     = "DONE"


class VisualServoBalloon:
    def __init__(self):
        rospy.init_node('visual_servo_balloon', anonymous=True)

        # ===================== 参数 =====================
        self.target_balloon = rospy.get_param('~target_balloon', 'red_balloon')

        # 图像尺寸
        self.img_w = rospy.get_param('~image_width',  640)
        self.img_h = rospy.get_param('~image_height', 480)
        self.img_cx = self.img_w / 2.0
        self.img_cy = self.img_h / 2.0

        # PD 增益 —— 偏航 (像素误差 → yaw_rate)
        self.k_p_yaw = rospy.get_param('~k_p_yaw', 0.003)
        self.k_d_yaw = rospy.get_param('~k_d_yaw', 0.008)
        self.deadzone_yaw = rospy.get_param('~deadzone_yaw', 8.0)

        # PD 增益 —— 高度 VZ (垂直居中)
        self.k_p_vz = rospy.get_param('~k_p_vz', 0.002)
        self.k_d_vz = rospy.get_param('~k_d_vz', 0.006)
        self.deadzone_vz = rospy.get_param('~deadzone_vz', 10.0)
        self.max_vz = rospy.get_param('~max_vz', 0.3)

        # 速度限制
        self.forward_velocity = rospy.get_param('~forward_velocity', 0.35)
        self.max_yaw_rate = rospy.get_param('~max_yaw_rate', 0.5)

        # 对齐判定
        self.align_threshold = rospy.get_param('~align_threshold', 15.0)

        # 冲刺参数
        self.area_ratio_threshold = rospy.get_param('~area_ratio_threshold', 0.08)
        self.sprint_velocity = rospy.get_param('~sprint_velocity', 1.5)
        self.sprint_time = rospy.get_param('~sprint_time', 1.2)

        # 搜索 & 丢失
        self.search_yaw_rate = rospy.get_param('~search_yaw_rate', 0.3)
        self.lost_timeout = rospy.get_param('~lost_timeout', 3.0)

        # ===================== 状态变量 =====================
        self.state = State.SEARCH
        self.last_det_time = 0.0
        self.sprint_start_time = 0.0

        # 误差历史 (用于 D 项)
        self.prev_err_yaw = 0.0
        self.prev_err_vz = 0.0
        self.prev_time = rospy.get_time()

        # 最新检测结果
        self.det_cx = 0.0   # 目标中心 x (像素)
        self.det_cy = 0.0   # 目标中心 y (像素)
        self.det_area_ratio = 0.0  # 目标面积占图像比

        # ===================== ROS 通信 =====================
        self.cmd_pub = rospy.Publisher(
            '/mavros/setpoint_raw/local', PositionTarget, queue_size=5)

        rospy.Subscriber('/objects', Objects, self.det_callback, queue_size=5)
        rospy.Subscriber(
            rospy.get_param('~odom_topic', '/rflysim/uav1/local/odom'),
            Odometry, self.odom_callback, queue_size=5)

        # 控制定时器 (30 Hz)
        self.control_rate = rospy.get_param('~control_rate', 30.0)
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.control_rate), self.control_loop)

        rospy.loginfo("=" * 50)
        rospy.loginfo("[VisualServo] 气球视觉伺服节点已启动")
        rospy.loginfo("[VisualServo] 目标: %s", self.target_balloon)
        rospy.loginfo("[VisualServo] 前进速度=%.2f  冲刺速度=%.2f  冲刺时间=%.1fs",
                      self.forward_velocity, self.sprint_velocity, self.sprint_time)
        rospy.loginfo("[VisualServo] k_p_yaw=%.4f  k_d_yaw=%.4f  deadzone=%.1f  max_rate=%.2f",
                      self.k_p_yaw, self.k_d_yaw, self.deadzone_yaw, self.max_yaw_rate)
        rospy.loginfo("[VisualServo] 控制模式: body VX + VZ + yaw_rate")
        rospy.loginfo("=" * 50)

    # ---------------------------------------------------------------
    #  回调
    # ---------------------------------------------------------------
    def det_callback(self, msg):
        """处理 HSV 检测结果，提取目标气球信息"""
        best = None
        best_area = 0

        for obj in msg.objects:
            if obj.class_name != self.target_balloon:
                continue
            w = obj.right_bottom_x - obj.left_top_x
            h = obj.right_bottom_y - obj.left_top_y
            area = w * h
            if area > best_area:
                best_area = area
                best = obj

        if best is not None:
            self.det_cx = best.center_x
            self.det_cy = best.center_y
            w = best.right_bottom_x - best.left_top_x
            h = best.right_bottom_y - best.left_top_y
            self.det_area_ratio = float(w * h) / (self.img_w * self.img_h)
            self.last_det_time = rospy.get_time()

    def odom_callback(self, msg):
        """预留：可用于高度保持等扩展"""
        pass

    # ---------------------------------------------------------------
    #  控制循环
    # ---------------------------------------------------------------
    def control_loop(self, event):
        now = rospy.get_time()
        dt = now - self.prev_time
        if dt < 1e-6:
            dt = 1.0 / self.control_rate
        self.prev_time = now

        target_visible = (now - self.last_det_time) < self.lost_timeout

        # ---------- 状态机 ----------
        if self.state == State.DONE:
            self._publish_cmd(0.0, 0.0, 0.0)
            return

        if self.state == State.SPRINT:
            if (now - self.sprint_start_time) >= self.sprint_time:
                rospy.loginfo("[VisualServo] ★ 冲刺完成！进入 DONE 状态")
                self.state = State.DONE
                self._publish_cmd(0.0, 0.0, 0.0)
                return
            else:
                yaw_rate = self._compute_yaw_rate(dt) if target_visible else 0.0
                vz = self._compute_vz(dt) if target_visible else 0.0
                self._publish_cmd(self.sprint_velocity, yaw_rate, vz)
                rospy.loginfo_throttle(0.3, "[VisualServo] SPRINT vx=%.2f yaw_rate=%.3f t=%.1fs",
                                       self.sprint_velocity, yaw_rate,
                                       now - self.sprint_start_time)
                return

        if not target_visible:
            if self.state != State.SEARCH:
                rospy.logwarn("[VisualServo] 目标丢失 %.1fs，进入 SEARCH",
                             now - self.last_det_time)
                self.state = State.SEARCH
            self._publish_cmd(0.0, self.search_yaw_rate, 0.0)
            return

        # ---- 目标可见 ----
        err_yaw = self.det_cx - self.img_cx

        if self.state == State.SEARCH:
            self.state = State.ALIGN
            rospy.loginfo("[VisualServo] 发现目标，进入 ALIGN")

        if self.state == State.ALIGN:
            yaw_rate = self._compute_yaw_rate(dt)
            vz = self._compute_vz(dt)
            self._publish_cmd(0.0, yaw_rate, vz)

            if abs(err_yaw) < self.align_threshold:
                rospy.loginfo("[VisualServo] 水平对齐完成 (err=%.1f)，进入 APPROACH", err_yaw)
                self.state = State.APPROACH

        elif self.state == State.APPROACH:
            yaw_rate = self._compute_yaw_rate(dt)
            vz = self._compute_vz(dt)

            # 根据对齐程度调节前进速度 (越偏越慢)
            align_ratio = max(0.0, 1.0 - abs(err_yaw) / (self.img_w * 0.3))
            vx = self.forward_velocity * align_ratio

            self._publish_cmd(vx, yaw_rate, vz)

            rospy.loginfo_throttle(0.5,
                "[VisualServo] APPROACH vx=%.2f(%.0f%%) yaw_rate=%.3f vz=%.3f err_x=%.1f area=%.3f",
                vx, align_ratio * 100, yaw_rate, vz, err_yaw, self.det_area_ratio)

            # 对齐偏差过大 → 回到 ALIGN
            if abs(err_yaw) > self.align_threshold * 3:
                rospy.logwarn("[VisualServo] 对齐偏差过大 (%.1f)，回到 ALIGN", err_yaw)
                self.state = State.ALIGN

            # 触发冲刺
            if self.det_area_ratio >= self.area_ratio_threshold:
                rospy.loginfo("[VisualServo] ★ 面积占比 %.3f >= %.3f，触发 SPRINT！",
                              self.det_area_ratio, self.area_ratio_threshold)
                self.state = State.SPRINT
                self.sprint_start_time = now

    # ---------------------------------------------------------------
    #  PD 控制器
    # ---------------------------------------------------------------
    def _compute_yaw_rate(self, dt):
        """
        计算 yaw_rate：目标在图像右侧 → err > 0 → 需要右转 → yaw_rate < 0
        (NED 坐标系下，yaw_rate > 0 为逆时针/左转)
        """
        err = self.det_cx - self.img_cx

        # 死区
        if abs(err) < self.deadzone_yaw:
            err_after_dz = 0.0
        else:
            err_after_dz = err - np.sign(err) * self.deadzone_yaw

        # PD
        d_err = (err_after_dz - self.prev_err_yaw) / dt if dt > 1e-6 else 0.0
        yaw_rate = -(self.k_p_yaw * err_after_dz + self.k_d_yaw * d_err)

        # 限幅
        yaw_rate = np.clip(yaw_rate, -self.max_yaw_rate, self.max_yaw_rate)

        self.prev_err_yaw = err_after_dz
        return yaw_rate

    def _compute_vz(self, dt):
        """
        计算垂直速度：目标在图像下方 → err > 0 → 需要下降
        MAVROS body frame: vz 正方向取决于飞控 (NED: 正=向下, ENU: 正=向上)
        这里取负号，使得目标在下方时 vz < 0 (向下移动/在ENU中)
        请根据实际飞控调整符号
        """
        err = self.det_cy - self.img_cy

        # 死区
        if abs(err) < self.deadzone_vz:
            err_after_dz = 0.0
        else:
            err_after_dz = err - np.sign(err) * self.deadzone_vz

        # PD
        d_err = (err_after_dz - self.prev_err_vz) / dt if dt > 1e-6 else 0.0
        vz = -(self.k_p_vz * err_after_dz + self.k_d_vz * d_err)

        # 限幅
        vz = np.clip(vz, -self.max_vz, self.max_vz)

        self.prev_err_vz = err_after_dz
        return vz

    # ---------------------------------------------------------------
    #  发布指令
    # ---------------------------------------------------------------
    def _publish_cmd(self, vx, yaw_rate, vz=0.0):
        """
        发布 body 系速度指令
          vx:       body 前向速度 (m/s)
          yaw_rate: 偏航角速度 (rad/s)
          vz:       垂直速度 (m/s)
        """
        cmd = PositionTarget()
        cmd.header.stamp = rospy.Time.now()
        cmd.header.frame_id = 'base_link'

        # FRAME_BODY_NED: body 坐标系
        cmd.coordinate_frame = PositionTarget.FRAME_BODY_NED

        # type_mask: 使用 vx, vz, yaw_rate；忽略位置、加速度、vy、yaw
        cmd.type_mask = (
            PositionTarget.IGNORE_PX |
            PositionTarget.IGNORE_PY |
            PositionTarget.IGNORE_PZ |
            PositionTarget.IGNORE_AFX |
            PositionTarget.IGNORE_AFY |
            PositionTarget.IGNORE_AFZ |
            PositionTarget.IGNORE_YAW |
            PositionTarget.IGNORE_VY
        )

        cmd.velocity.x = vx
        cmd.velocity.y = 0.0
        cmd.velocity.z = vz
        cmd.yaw_rate = yaw_rate

        self.cmd_pub.publish(cmd)

    # ---------------------------------------------------------------
    def spin(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        node = VisualServoBalloon()
        node.spin()
    except rospy.ROSInterruptException:
        pass
