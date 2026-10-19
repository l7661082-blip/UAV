/******************************************************************************
 * @file       visual_servo.cpp
 * @brief      Visual Servo 行为树节点 - 三阶段控制（body系 VX+VZ+yaw_rate）
 *
 * @details    三阶段：对齐(ALIGN) -> 靠近(APPROACH) -> 冲刺(SPRINT)
 *             全程使用 publishVelocityWithYawRate（body 坐标系）
 *             控制变量：VX(前进) + VZ(升降) + yaw_rate(偏航角速度)
 *
 * @author     郭定<1604289397@qq.com>
 * @date       2026-03-16
 ******************************************************************************/
#include "nodes/action/visual_servo.h"
#include <algorithm>
#include <cmath>


VisualServo::VisualServo(const std::string& name, const NodeConfig& config)
    : DroneActionNode(name, config), 
      current_state_(ALIGN),
      has_ever_received_data_(false),
      max_area_ratio_(0.0) {
    auto nh = mct->getROSHandle();
    nh->param<double>("cx", img_cx_, 320.0);
    nh->param<double>("cy", img_cy_, 240.0);
    nh->param<int>("img_w", img_w_, 640);
    nh->param<int>("img_h", img_h_, 480);
}

PortsList VisualServo::providedPorts() {
    return { InputPort<PixelTarget>("pixel_target", "像素目标"),
             
             // yaw_rate PD 控制
             InputPort<double>("k_p_yaw_rate", 0.003, "偏航角速度 P 增益 (rad/s per pixel)"),
             InputPort<double>("k_d_yaw_rate", 0.008, "偏航角速度 D 增益"),
             InputPort<double>("max_yaw_rate", 0.5, "最大偏航角速度 (rad/s)"),
             InputPort<double>("deadzone_yaw", 8.0, "偏航死区 (pixel)"),
             
             // VZ PD 控制
             InputPort<double>("k_p_vz", 0.002, "垂直速度 P 增益"),
             InputPort<double>("k_d_vz", 0.006, "垂直速度 D 增益"),
             InputPort<double>("max_vz", 0.3, "最大垂直速度 (m/s)"),
             InputPort<double>("deadzone_vz", 10.0, "垂直死区 (pixel)"),
             
             // 速度参数
             InputPort<double>("forward_velocity", 0.35, "逼近阶段前进速度 (m/s)"),
             InputPort<double>("sprint_velocity", 1.5, "冲刺速度 (m/s)"),
             InputPort<double>("sprint_time", 1.2, "冲刺持续时间 (s)"),
             
             // 其他
             InputPort<double>("data_timeout", 1.0, "数据超时 (s)"),
             InputPort<double>("approach_area_threshold", 0.08, "触发冲刺的面积占比"),
             InputPort<double>("center_tolerance", 0.15, "对齐判定区域比例"),
             InputPort<double>("search_yaw_rate", 0.3, "搜索模式旋转速度 (rad/s)") };
}

NodeStatus VisualServo::onStart() {
    // 加载参数
    k_p_yaw_rate_ = getInput<double>("k_p_yaw_rate").value();
    k_d_yaw_rate_ = getInput<double>("k_d_yaw_rate").value();
    max_yaw_rate_ = getInput<double>("max_yaw_rate").value();
    deadzone_yaw_ = getInput<double>("deadzone_yaw").value();
    
    k_p_vz_ = getInput<double>("k_p_vz").value();
    k_d_vz_ = getInput<double>("k_d_vz").value();
    max_vz_ = getInput<double>("max_vz").value();
    deadzone_vz_ = getInput<double>("deadzone_vz").value();
    
    forward_velocity_ = getInput<double>("forward_velocity").value();
    sprint_velocity_ = getInput<double>("sprint_velocity").value();
    sprint_time_ = getInput<double>("sprint_time").value();
    
    data_timeout_ = getInput<double>("data_timeout").value();
    approach_area_threshold_ = getInput<double>("approach_area_threshold").value();
    center_tolerance_ = getInput<double>("center_tolerance").value();
    search_yaw_rate_ = getInput<double>("search_yaw_rate").value();
    
    // 初始化状态机
    current_state_ = APPROACHING;  // 直接进入边飞边对齐模式（气球是移动目标）
    has_ever_received_data_ = false;
    max_area_ratio_ = 0.0;
    align_stable_frames_ = 0;
    
    // 初始化微分状态
    prev_err_x_ = 0.0;
    prev_err_y_ = 0.0;
    prev_time_ = ros::Time::now();
    
    updateTargetData();
    
    ROS_INFO("[%s]: Visual Servo Started (VX+VZ+yaw_rate)", name().c_str());
    ROS_INFO("[%s]: k_p_yaw_rate=%.4f k_d=%.4f max_rate=%.2f deadzone=%.1f", 
             name().c_str(), k_p_yaw_rate_, k_d_yaw_rate_, max_yaw_rate_, deadzone_yaw_);
    ROS_INFO("[%s]: forward_vel=%.2f sprint_vel=%.2f sprint_time=%.1f area_thresh=%.3f",
             name().c_str(), forward_velocity_, sprint_velocity_, sprint_time_, approach_area_threshold_);
    
    return NodeStatus::RUNNING;
}

NodeStatus VisualServo::onRunning() {
    updateTargetData();
    
    // 计算时间步长
    ros::Time now = ros::Time::now();
    double dt = (now - prev_time_).toSec();
    if (dt < 0.001) dt = 0.001;
    
    // 目标丢失处理（冲刺阶段不检查）
    if (current_state_ != SPRINTING && isTargetLost()) {
        if (max_area_ratio_ > approach_area_threshold_) {
            sendHoverCommand();
            ROS_INFO("[%s]: Target lost after approach (max_ratio=%.3f). SUCCESS.", 
                     name().c_str(), max_area_ratio_);
            return NodeStatus::SUCCESS;
        }
        // 搜索模式：原地慢转
        publishVelocityWithYawRate(0, 0, 0, search_yaw_rate_);
        ROS_WARN_THROTTLE(1.0, "[%s]: Target lost. Searching (yaw_rate=%.2f)...", 
                          name().c_str(), search_yaw_rate_);
        return NodeStatus::RUNNING;
    }
    
    // 计算基本量
    double img_area = static_cast<double>(img_w_) * img_h_;
    double current_ratio = last_target_.area() / img_area;
    double err_x = last_target_.center_x - img_cx_;  // 正=目标在右侧
    double err_y = last_target_.center_y - img_cy_;  // 正=目标在下方
    
    // 更新最大面积记录
    if (current_ratio > max_area_ratio_) {
        max_area_ratio_ = current_ratio;
    }
    
    // PD 控制计算
    double yaw_rate = computeYawRate(err_x, dt);
    double vz = computeVz(err_y, dt);
    
    // 更新微分状态
    prev_time_ = now;
    
    // =========== 三阶段状态机 ===========
    switch (current_state_) {
        case ALIGN: {
            // 阶段1：对齐 — 原地旋转 + 升降，使目标居中
            publishVelocityWithYawRate(0, 0, vz, yaw_rate);
            
            if (isCentered()) {
                align_stable_frames_++;
                if (align_stable_frames_ >= ALIGN_STABLE_FRAMES_REQUIRED) {
                    current_state_ = APPROACHING;
                    align_stable_frames_ = 0;
                    ROS_INFO("[%s]: Aligned! err:(%.0f,%.0f) -> APPROACHING", 
                             name().c_str(), err_x, err_y);
                }
            } else {
                align_stable_frames_ = 0;
            }
            
            ROS_INFO_THROTTLE(1.0, "[%s]: ALIGN - err:(%.0f,%.0f) yaw_rate:%.3f vz:%.3f stable:%d", 
                             name().c_str(), err_x, err_y, yaw_rate, vz, align_stable_frames_);
            break;
        }
        
        case APPROACHING: {
            // 阶段2：靠近 — VX 前进 + yaw_rate + VZ，全程持续修正
            // 根据对齐程度调节前进速度（越偏越慢）
            double align_ratio = std::max(0.0, 1.0 - std::abs(err_x) / (img_w_ * 0.3));
            double vx = forward_velocity_ * align_ratio;
            
            publishVelocityWithYawRate(vx, 0, vz, yaw_rate);
            
            // 对齐偏差过大 → 回到 ALIGN
            if (std::abs(err_x) > img_w_ * center_tolerance_ * 3) {
                current_state_ = ALIGN;
                align_stable_frames_ = 0;
                ROS_WARN("[%s]: Alignment lost (err_x=%.0f), back to ALIGN", 
                         name().c_str(), err_x);
                break;
            }
            
            // 面积达阈值 → 触发冲刺
            if (current_ratio >= approach_area_threshold_) {
                current_state_ = SPRINTING;
                sprint_start_time_ = ros::Time::now();
                ROS_INFO("[%s]: Area %.1f%% >= %.1f%% -> SPRINT! (sprint_vel=%.2f, time=%.1fs)", 
                         name().c_str(), current_ratio*100, approach_area_threshold_*100,
                         sprint_velocity_, sprint_time_);
            }
            
            ROS_INFO_THROTTLE(0.5, "[%s]: APPROACH - vx:%.2f(%.0f%%) yaw_rate:%.3f vz:%.3f err:(%.0f,%.0f) area:%.1f%%", 
                             name().c_str(), vx, align_ratio*100, yaw_rate, vz, 
                             err_x, err_y, current_ratio*100);
            break;
        }
        
        case SPRINTING: {
            // 阶段3：冲刺 — 全速前进，保持 yaw_rate 和 VZ 修正
            double sprint_dt = (ros::Time::now() - sprint_start_time_).toSec();
            
            // 超时 → 完成
            if (sprint_dt >= sprint_time_) {
                sendHoverCommand();
                ROS_INFO("[%s]: Sprint completed (%.1fs). SUCCESS.", name().c_str(), sprint_dt);
                return NodeStatus::SUCCESS;
            }
            
            // 冲刺中仍然持续修正 yaw_rate 和 vz（如果还看得到目标）
            if (!isTargetLost()) {
                publishVelocityWithYawRate(sprint_velocity_, 0, vz, yaw_rate);
            } else {
                // 目标丢失，保持直线冲刺
                publishVelocityWithYawRate(sprint_velocity_, 0, 0, 0);
            }
            
            ROS_INFO_THROTTLE(0.2, "[%s]: SPRINT - vx:%.2f yaw_rate:%.3f vz:%.3f (%.1f/%.1fs)", 
                             name().c_str(), sprint_velocity_, yaw_rate, vz,
                             sprint_dt, sprint_time_);
            break;
        }
    }
    
    return NodeStatus::RUNNING;
}

// =====================================================================
// PD 控制器
// =====================================================================

double VisualServo::computeYawRate(double err_x, double dt) {
    // 目标在右侧(err_x>0) → 需要右转 → yaw_rate < 0 (NED 逆时针为正)
    double err = err_x;
    
    // 死区
    double err_after_dz = 0.0;
    if (std::abs(err) > deadzone_yaw_) {
        err_after_dz = err - std::copysign(deadzone_yaw_, err);
    }
    
    // PD
    double d_err = (dt > 1e-6) ? (err_after_dz - prev_err_x_) / dt : 0.0;
    double yaw_rate = -(k_p_yaw_rate_ * err_after_dz + k_d_yaw_rate_ * d_err);
    
    // 限幅
    yaw_rate = std::clamp(yaw_rate, -max_yaw_rate_, max_yaw_rate_);
    
    prev_err_x_ = err_after_dz;
    return yaw_rate;
}

double VisualServo::computeVz(double err_y, double dt) {
    // 目标在下方(err_y>0) → 需要下降 → vz < 0 (看飞控坐标系)
    double err = err_y;
    
    // 死区
    double err_after_dz = 0.0;
    if (std::abs(err) > deadzone_vz_) {
        err_after_dz = err - std::copysign(deadzone_vz_, err);
    }
    
    // PD
    double d_err = (dt > 1e-6) ? (err_after_dz - prev_err_y_) / dt : 0.0;
    double vz = -(k_p_vz_ * err_after_dz + k_d_vz_ * d_err);
    
    // 限幅
    vz = std::clamp(vz, -max_vz_, max_vz_);
    
    prev_err_y_ = err_after_dz;
    return vz;
}

// =====================================================================
// 辅助函数
// =====================================================================

void VisualServo::updateTargetData() {
    auto target_res = getInput<PixelTarget>("pixel_target");
    if (!target_res) return;

    PixelTarget current_target = target_res.value();
    if (current_target.is_detected) {
        last_target_ = current_target;
        last_valid_detection_time_ = ros::Time::now(); 
        has_ever_received_data_ = true;
        
        double img_area = static_cast<double>(img_w_) * img_h_;
        double ratio = last_target_.area() / img_area;
        if (ratio > max_area_ratio_) {
            max_area_ratio_ = ratio;
        }
    }
}

bool VisualServo::isTargetLost() {
    if (!has_ever_received_data_) return true;
    return (ros::Time::now() - last_valid_detection_time_).toSec() > data_timeout_;
}

bool VisualServo::isCentered() const {
    double err_x = last_target_.center_x - img_cx_;
    double err_y = last_target_.center_y - img_cy_;
    double tol_w = last_target_.width * center_tolerance_;
    double tol_h = last_target_.height * center_tolerance_;
    return std::abs(err_x) < tol_w && std::abs(err_y) < tol_h;
}

BT_REGISTER_NODES(factory) {
    factory.registerNodeType<VisualServo>("VisualServo");
}
