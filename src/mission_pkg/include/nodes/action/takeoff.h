/******************************************************************************
 * @file       takeoff.h
 * @brief      Takeoff 行为树节点声明
 *
 * @author     郭定<1604289397@qq.com>
 * @date       2026-01-06
 ******************************************************************************/
#ifndef TAKEOFF_H
#define TAKEOFF_H

#include "base/drone_action_node.h"
#include "utils/pose_utils.h"

class Takeoff : public DroneActionNode {
public:
    Takeoff(const std::string& name, const NodeConfig& config);
    
    static PortsList providedPorts();

    virtual NodeStatus onStart() override;
    virtual NodeStatus onRunning() override;

private:
    Position3D goal_pos;
    bool is_rc_control;
    double tolerance;
    double takeoff_yaw_;  // 起飞时的航向（ENU yaw）
};

#endif // TAKEOFF_H
