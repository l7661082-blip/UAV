/******************************************************************************
 * @file       drone_action_node.cpp
 * @brief      行为树节点基类实现
 *
 * @author     郭定<1604289397@qq.com>
 * @date       2026-01-06
 ******************************************************************************/
#include "base/drone_action_node.h"
#include "utils/pose_utils.h"

DroneActionNode::DroneActionNode(const std::string& name, const NodeConfig& config)
    : StatefulActionNode(name, config) {
    mct = MavRosConnect::getInstance();
    fcu_odom_ptr = mct->getFcuOdom();
    fcu_state_ptr = mct->getFcuState();
    pos_target_pub = *mct->getPosTgtPublisher();
    set_mode_client = mct->getSetModeClient();
    arming_client = mct->getArmClient();
    last_request_time = ros::Time(0);
}

void DroneActionNode::onHalted() {
    ROS_INFO("[%s]: Halted, entering hover mode.", name().c_str());
    sendHoverCommand();
}

void DroneActionNode::publishVelocityWithYawRate(double vx, double vy, double vz, double yaw_rate) {
    hover_active_ = false;
    
    mavros_msgs::PositionTarget cmd;
    cmd.header.stamp = ros::Time::now();
    cmd.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
    
    cmd.velocity.x = vx;
    cmd.velocity.y = vy;
    cmd.velocity.z = vz;
    cmd.yaw_rate = yaw_rate;
    
    cmd.type_mask = mavros_msgs::PositionTarget::IGNORE_PX |
                    mavros_msgs::PositionTarget::IGNORE_PY |
                    mavros_msgs::PositionTarget::IGNORE_PZ |
                    mavros_msgs::PositionTarget::IGNORE_YAW |
                    mavros_msgs::PositionTarget::IGNORE_AFX |
                    mavros_msgs::PositionTarget::IGNORE_AFY |
                    mavros_msgs::PositionTarget::IGNORE_AFZ;
    
    mct->updateCommand(cmd);
}

void DroneActionNode::publishVelocityWithYaw(double vx_body, double vy_body, double vz, double yaw) {
    hover_active_ = false;
    
    mavros_msgs::PositionTarget cmd;
    cmd.header.stamp = ros::Time::now();
    cmd.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;  // 使用全局坐标系
    
    // 将机体速度转换为全局速度
    if (fcu_odom_ptr && fcu_odom_ptr->header.stamp != ros::Time(0)) {
        auto& q = fcu_odom_ptr->pose.pose.orientation;
        double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        double current_yaw = std::atan2(siny_cosp, cosy_cosp);
        
        // 2D 旋转变换：机体坐标系 -> 全局坐标系
        double cos_yaw = std::cos(current_yaw);
        double sin_yaw = std::sin(current_yaw);
        cmd.velocity.x = vx_body * cos_yaw - vy_body * sin_yaw;
        cmd.velocity.y = vx_body * sin_yaw + vy_body * cos_yaw;
        cmd.velocity.z = vz;  // Z轴不需要转换
    } else {
        // 没有里程计数据，无法转换坐标系，使用零速度
        ROS_WARN_THROTTLE(1.0, "No valid odometry for velocity transformation, using zero velocity");
        cmd.velocity.x = 0;
        cmd.velocity.y = 0;
        cmd.velocity.z = 0;
    }
    
    cmd.yaw = yaw;  // 全局yaw角度
    
    cmd.type_mask = mavros_msgs::PositionTarget::IGNORE_PX |
                    mavros_msgs::PositionTarget::IGNORE_PY |
                    mavros_msgs::PositionTarget::IGNORE_PZ |
                    mavros_msgs::PositionTarget::IGNORE_YAW_RATE |
                    mavros_msgs::PositionTarget::IGNORE_AFX |
                    mavros_msgs::PositionTarget::IGNORE_AFY |
                    mavros_msgs::PositionTarget::IGNORE_AFZ;
    
    mct->updateCommand(cmd);
}


void DroneActionNode::publishPosition(const geometry_msgs::Point& pos, double yaw, uint16_t coordinate_frame) {
    hover_active_ = false;  // 重置悬停状态

    mavros_msgs::PositionTarget cmd;
    cmd.header.stamp = ros::Time::now();
    cmd.header.frame_id = name();
    cmd.coordinate_frame = coordinate_frame;
    cmd.type_mask = mavros_msgs::PositionTarget::IGNORE_VX |
                    mavros_msgs::PositionTarget::IGNORE_VY |
                    mavros_msgs::PositionTarget::IGNORE_VZ |
                    mavros_msgs::PositionTarget::IGNORE_YAW_RATE |
                    mavros_msgs::PositionTarget::IGNORE_AFX |
                    mavros_msgs::PositionTarget::IGNORE_AFY |
                    mavros_msgs::PositionTarget::IGNORE_AFZ |
                    mavros_msgs::PositionTarget::FORCE;
    
    // ENU → NED: NED_x = ENU_y, NED_y = -ENU_x, yaw += 90°
    cmd.position.x = -pos.y;    // NED x = -ENU y
    cmd.position.y = pos.x;   // NED y = ENU x
    cmd.position.z = pos.z;
    cmd.yaw = yaw + M_PI / 2.0;  // ENU yaw → NED yaw
    mct->updateCommand(cmd);
}

void DroneActionNode::sendHoverCommand() {
    if (fcu_odom_ptr->header.stamp == ros::Time(0)) return;

    // 如果未进入悬停状态，则捕获当前位置作为悬停点
    if (!hover_active_) {
        hover_pos_ = fcu_odom_ptr->pose.pose.position;
        hover_yaw_ = PoseUtils::getYawFromQuat(fcu_odom_ptr->pose.pose.orientation);
        hover_active_ = true;
        ROS_INFO("[%s]: Maintaining hover at (%.2f, %.2f, %.2f) yaw: %.2f", 
                 name().c_str(), hover_pos_.x, hover_pos_.y, hover_pos_.z, hover_yaw_);
    }

    // 直接构造命令发送，避免调用 publishPosition 重置标志
    mavros_msgs::PositionTarget cmd;
    cmd.header.stamp = ros::Time::now();
    cmd.header.frame_id = name() + "_hover";
    cmd.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    cmd.type_mask = mavros_msgs::PositionTarget::IGNORE_VX |
                    mavros_msgs::PositionTarget::IGNORE_VY |
                    mavros_msgs::PositionTarget::IGNORE_VZ |
                    mavros_msgs::PositionTarget::IGNORE_YAW_RATE |
                    mavros_msgs::PositionTarget::IGNORE_AFX |
                    mavros_msgs::PositionTarget::IGNORE_AFY |
                    mavros_msgs::PositionTarget::IGNORE_AFZ |
                    mavros_msgs::PositionTarget::FORCE;
    
    // odom 是 ENU，发送需要 ENU→NED（与 publishPosition 保持一致）
    cmd.position.x = -hover_pos_.y;   // NED x = -ENU y
    cmd.position.y = hover_pos_.x;    // NED y = ENU x
    cmd.position.z = hover_pos_.z;
    cmd.yaw = hover_yaw_ + M_PI / 2.0;
    mct->updateCommand(cmd);
}

bool DroneActionNode::throttledSetMode(const std::string& mode, double interval_sec) {
    ros::Time now = ros::Time::now();
    if (fcu_state_ptr->mode != mode && (now - last_request_time > ros::Duration(interval_sec))) {
        mavros_msgs::SetMode mode_msg;
        mode_msg.request.custom_mode = mode;
        if (set_mode_client->call(mode_msg) && mode_msg.response.mode_sent) {
            ROS_INFO("[%s]: Requesting mode %s", name().c_str(), mode.c_str());
        }
        last_request_time = now;
        return true;
    }
    return false;
}

bool DroneActionNode::throttledArm(bool value, double interval_sec) {
    ros::Time now = ros::Time::now();
    if (fcu_state_ptr->armed != value && (now - last_request_time > ros::Duration(interval_sec))) {
        mavros_msgs::CommandBool arm_msg;
        arm_msg.request.value = value;
        if (arming_client->call(arm_msg) && arm_msg.response.success) {
            ROS_INFO("[%s]: Requesting %s", name().c_str(), value ? "ARM" : "DISARM");
        }
        last_request_time = now;
        return true;
    }
    return false;
}

bool DroneActionNode::checkArrival(const geometry_msgs::Point& target, double tolerance,
                                   double target_yaw, double yaw_tolerance) {
    // 1. 检查位置误差
    // odom 和 target 都是 ENU，直接比较
    double dx = fcu_odom_ptr->pose.pose.position.x - target.x;
    double dy = fcu_odom_ptr->pose.pose.position.y - target.y;
    double dz = fcu_odom_ptr->pose.pose.position.z - target.z;
    double dist = std::sqrt(dx*dx + dy*dy + dz*dz);
    
    ROS_INFO_THROTTLE(2.0, "[checkArrival] odom=(%.2f,%.2f,%.2f) target=(%.2f,%.2f,%.2f) dist=%.3f tol=%.2f",
                     fcu_odom_ptr->pose.pose.position.x, fcu_odom_ptr->pose.pose.position.y, fcu_odom_ptr->pose.pose.position.z,
                     target.x, target.y, target.z, dist, tolerance);
    
    if (dist > tolerance) return false;
    
    // 2. 检查偏航误差 (如果指定了 target_yaw)
    if (!std::isnan(target_yaw)) {
        double current_yaw = PoseUtils::getYawFromQuat(fcu_odom_ptr->pose.pose.orientation);
        double yaw_diff = target_yaw - current_yaw;
        ROS_INFO_THROTTLE(2.0, "[checkArrival] yaw_diff=(%.2f) target=(%.2f) current=(%.2f) yaw_tolerance=(%.2f)    ",
                     yaw_diff, target_yaw, current_yaw, yaw_tolerance);
        // 归一化到 [-sigma, sigma]
        while (yaw_diff > M_PI) yaw_diff -= 2 * M_PI;
        while (yaw_diff < -M_PI) yaw_diff += 2 * M_PI;
        
        if (std::abs(yaw_diff) > yaw_tolerance) return false;
    }
    
    return true;
}
