/******************************************************************************
 * @file       visual_servo.h
 * @brief      Visual Servo - 三阶段控制（body系 VX+VZ+yaw_rate 控制版）
 * 
 * @details    ALIGN → APPROACH → SPRINT
 *             全程使用 publishVelocityWithYawRate (body 坐标系)
 ******************************************************************************/
#ifndef VISUAL_SERVO_H
#define VISUAL_SERVO_H

#include "base/drone_action_node.h"
#include <ros/time.h>

class VisualServo : public DroneActionNode {
public:
    VisualServo(const std::string& name, const NodeConfig& config);
    static PortsList providedPorts();
    virtual NodeStatus onStart() override;
    virtual NodeStatus onRunning() override;

private:
    // 三阶段状态机
    enum ServoState {
        ALIGN,      // 对齐：原地旋转 yaw_rate，使目标水平居中
        APPROACHING, // 靠近：VX 前进 + yaw_rate + VZ 修正
        SPRINTING   // 冲刺：全速 VX 冲刺一段固定时间
    };
    
    // 状态
    ServoState current_state_;
    PixelTarget last_target_;
    ros::Time last_valid_detection_time_;
    bool has_ever_received_data_;
    double max_area_ratio_;
    ros::Time sprint_start_time_;
    int align_stable_frames_;
    
    // 相机参数
    double img_cx_, img_cy_;
    int img_w_, img_h_;
    
    // yaw_rate PD 控制参数
    double k_p_yaw_rate_;   // 偏航角速度 P 增益 (rad/s per pixel)
    double k_d_yaw_rate_;   // 偏航角速度 D 增益
    double max_yaw_rate_;   // 最大偏航角速度 (rad/s)
    double deadzone_yaw_;   // 偏航死区 (pixel)
    
    // VZ PD 控制参数
    double k_p_vz_;         // 垂直速度 P 增益
    double k_d_vz_;         // 垂直速度 D 增益
    double max_vz_;         // 最大垂直速度 (m/s)
    double deadzone_vz_;    // 垂直死区 (pixel)
    
    // 前进速度参数
    double forward_velocity_;       // 逼近阶段前进速度 (m/s)
    double sprint_velocity_;        // 冲刺速度 (m/s)
    double sprint_time_;            // 冲刺持续时间 (s)
    
    // 其他参数
    double data_timeout_;           // 数据超时 (s)
    double approach_area_threshold_;// 触发冲刺的面积占比阈值
    double center_tolerance_;       // 对齐判定区域比例
    double search_yaw_rate_;        // 搜索模式旋转速度 (rad/s)
    
    // 微分计算用
    double prev_err_x_, prev_err_y_;
    ros::Time prev_time_;
    
    // 常量
    static constexpr int ALIGN_STABLE_FRAMES_REQUIRED = 5;

    // 辅助函数
    void updateTargetData();
    bool isTargetLost();
    bool isCentered() const;
    
    // PD 控制器
    double computeYawRate(double err_x, double dt);
    double computeVz(double err_y, double dt);
};

#endif
