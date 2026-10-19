/******************************************************************************
 * @file       drone_action_node.h
 * @brief      行为树节点基类 - 封装无人机控制的通用功能
 *
 * @details    为所有无人机行为树节点提供统一的状态访问、控制指令发布等
 *
 * @author     郭定<1604289397@qq.com>
 * @date       2026-01-09
 ******************************************************************************/
#ifndef DRONE_ACTION_NODE_H
#define DRONE_ACTION_NODE_H

#include "utils/common.hpp"
#include "mavros_cnt.h"
#include <ros/ros.h>

using namespace BT;

class DroneActionNode : public StatefulActionNode {
public:
    DroneActionNode(const std::string& name, const NodeConfig& config);
    
    // 核心生命周期封装
    virtual NodeStatus onStart() override = 0;
    virtual NodeStatus onRunning() override = 0;
    virtual void onHalted() override;

protected:
    // 连接器与状态
    std::shared_ptr<MavRosConnect> mct;
    const nav_msgs::Odometry* fcu_odom_ptr;
    const mavros_msgs::State* fcu_state_ptr;
    ros::Publisher pos_target_pub;
    ros::ServiceClient* set_mode_client;
    ros::ServiceClient* arming_client;

    // 通用控制指令发布（机体坐标系）
    // 速度控制 + yaw_rate 控制（对齐阶段）
    void publishVelocityWithYawRate(double vx, double vy, double vz, double yaw_rate);
    
    // 速度控制 + yaw 角度控制（靠近/冲刺阶段）
    void publishVelocityWithYaw(double vx, double vy, double vz, double yaw);
    
    void publishPosition(const geometry_msgs::Point& pos, double yaw,
                        uint16_t coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED);

    void publishFullCommand(const mavros_msgs::PositionTarget& full_cmd);
    
    // 发送当前悬停指令
    void sendHoverCommand();

    // 辅助功能
    bool throttledSetMode(const std::string& mode, double interval_sec = 5.0);
    bool throttledArm(bool value, double interval_sec = 5.0);
    bool checkArrival(const geometry_msgs::Point& target, double tolerance = 0.2, 
                     double target_yaw = NAN, double yaw_tolerance = 0.1);

private:
    ros::Time last_request_time;
    
    // 悬停状态管理
    bool hover_active_{false};
    geometry_msgs::Point hover_pos_;
    double hover_yaw_{0.0};
};

#endif // DRONE_ACTION_NODE_H
