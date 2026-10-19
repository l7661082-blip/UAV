/******************************************************************************
 * @file       planning_control.h
 * @brief      Planning Control 行为树节点声明
 *
 * @author     郭定<1604289397@qq.com>
 * @date       2026-01-06
 ******************************************************************************/
#ifndef PLANNING_CONTROL_H
#define PLANNING_CONTROL_H

#include "base/drone_action_node.h"
#include <quadrotor_msgs/PositionCommand.h>

class PlanningControl : public DroneActionNode {
public:
    PlanningControl(const std::string& name, const NodeConfig& config);
    
    static PortsList providedPorts();

    virtual NodeStatus onStart() override;
    virtual NodeStatus onRunning() override;

private:
   void plannerCmdCallback(const quadrotor_msgs::PositionCommand::ConstPtr& msg);
   void initializeTarget(const Position3D& pos);

    Position3D target_pos_;
    double tolerance;
    double offset_z;
    bool control_yaw;
    double initial_yaw_{0.0};  // Starting yaw when control_yaw is false
    bool target_initialized;

    ros::Publisher planner_goal_pub;
    ros::Subscriber planner_cmd_sub;
    quadrotor_msgs::PositionCommand last_planner_cmd;
    bool has_planner_cmd;
};

#endif // PLANNING_CONTROL_H
