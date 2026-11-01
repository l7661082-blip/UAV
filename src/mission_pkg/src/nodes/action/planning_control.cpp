/******************************************************************************
 * @file       planning_control.cpp
 * @brief      Planning Control 行为树节点 - 规划器控制
 *
 * @details    与外部路径规划器交互，执行复杂导航任务
 *
 * @author     郭定<1604289397@qq.com>
 * @date       2026-01-06
 ******************************************************************************/
#include "nodes/action/planning_control.h"
#include "utils/pose_utils.h"
#include "utils/flight_boundary.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

PlanningControl::PlanningControl(const std::string& name, const NodeConfig& config)
    : DroneActionNode(name, config), has_planner_cmd(false) {
    
    auto nh = mct->getROSHandle();
    std::string goal_topic, cmd_topic;
    nh->param<std::string>("goal_topic", goal_topic, "/move_base_simple/goal");
    nh->param<std::string>("planner_cmd_topic", cmd_topic, "/planning/pos_cmd");

    planner_goal_pub = nh->advertise<geometry_msgs::PoseStamped>(goal_topic, 10);
    planner_cmd_sub = nh->subscribe(cmd_topic, 1, &PlanningControl::plannerCmdCallback, this);
}

PortsList PlanningControl::providedPorts() {
    return { InputPort<Position3D>("goal", "目标点(x;y;z)"),
             InputPort<double>("offset_z", 0.0, "沿当前航向的水平目标偏移，历史端口名"),
             InputPort<double>("tolerance", "到达容差"),
             InputPort<bool>("control_yaw", false, "是否控制偏航角"),
             OutputPort<Position3D>("final_position", "到达的最终位置(x;y;z)") };
}

void PlanningControl::plannerCmdCallback(const quadrotor_msgs::PositionCommand::ConstPtr& msg) {
    last_planner_cmd = *msg;
    has_planner_cmd = true;
}

void PlanningControl::initializeTarget(const Position3D& pos) {
    target_pos_ = pos;

    // 计算带有偏移的目标点 (沿着飞机当前位姿的机头方向投射)
    tf2::Quaternion q_curr;
    tf2::fromMsg(fcu_odom_ptr->pose.pose.orientation, q_curr);
    double r, p, yaw;
    tf2::Matrix3x3(q_curr).getRPY(r, p, yaw);

    double target_x = target_pos_.x + offset_z * cos(yaw);
    double target_y = target_pos_.y + offset_z * sin(yaw);
    double target_z = target_pos_.z;

    FlightBoundary boundary;
    if (!FlightBoundary::load(boundary)) {
        target_initialized = false;
        sendHoverCommand();
        return;
    }
    geometry_msgs::Point bounded_goal;
    bounded_goal.x = target_x;
    bounded_goal.y = target_y;
    bounded_goal.z = target_z;
    if (!boundary.clamp(bounded_goal, name())) {
        target_initialized = false;
        sendHoverCommand();
        return;
    }
    target_x = bounded_goal.x;
    target_y = bounded_goal.y;

    // 向规划器发送目标（都使用 ENU，直接发送）
    geometry_msgs::PoseStamped ps;
    ps.header.stamp = ros::Time::now();
    ps.header.frame_id = "world";
    ps.pose.position.x = target_x;
    ps.pose.position.y = target_y;
    ps.pose.position.z = target_z;
    
    tf2::Quaternion q_tgt;
    q_tgt.setRPY(0, 0, 0); 
    ps.pose.orientation = tf2::toMsg(q_tgt);

    planner_goal_pub.publish(ps);
    has_planner_cmd = false;

    ROS_INFO("[%s]: ENU goal to ego=(%.2f, %.2f, %.2f), yaw=%.2f, offset=%.2f",
             name().c_str(), ps.pose.position.x, ps.pose.position.y, ps.pose.position.z, yaw, offset_z);

    // 为了 checkArrival 判断，更新目标点
    target_pos_.x = target_x;
    target_pos_.y = target_y;

    target_initialized = true;
    ROS_INFO("[%s]: Target acquired! Planning navigation start -> (%.2f, %.2f, %.2f), offset: %.2f", 
             name().c_str(), target_x, target_y, target_z, offset_z);
}

NodeStatus PlanningControl::onStart() {
    auto off_res = getInput<double>("offset_z");
    offset_z = off_res.value();

    auto tol_res = getInput<double>("tolerance");
    if (tol_res) {
        tolerance = tol_res.value();
    } else {
        ros::NodeHandle nh;
        nh.param("tolerance", tolerance, 0.2);
    }

    auto yaw_res = getInput<bool>("control_yaw");
    control_yaw = yaw_res.value();

    ROS_INFO("[%s]: Initialized with offset_z=%.2f, tolerance=%.2f, control_yaw=%d",
             name().c_str(), offset_z, tolerance, control_yaw);

    // 捕获初始偏航角 (供 !control_yaw 使用)
    initial_yaw_ = PoseUtils::getYawFromQuat(fcu_odom_ptr->pose.pose.orientation);
    
    // 尝试获取目标点
    auto goal_res = getInput<Position3D>("goal");
    if (!goal_res) {
        ROS_INFO("[%s]: Waiting for goal...", name().c_str());
        target_initialized = false;
        sendHoverCommand();
        return NodeStatus::RUNNING;
    }

    // 初始化目标
    initializeTarget(goal_res.value());
    return target_initialized ? NodeStatus::RUNNING : NodeStatus::FAILURE;
}

NodeStatus PlanningControl::onRunning() {
    // 如果目标还未初始化，尝试获取
    if (!target_initialized) {
        auto goal_res = getInput<Position3D>("goal");
        if (!goal_res) {
            sendHoverCommand();
            return NodeStatus::RUNNING;
        }
        
        initializeTarget(goal_res.value());
        if (!target_initialized) return NodeStatus::FAILURE;
        ROS_INFO("[%s]: Target acquired!", name().c_str());
    }

    // 检查是否到达 (3D 距离)
    geometry_msgs::Point target;
    target.x = target_pos_.x;
    target.y = target_pos_.y;
    target.z = target_pos_.z;
    if (checkArrival(target, tolerance)) {
        ROS_INFO("[%s]: Planning navigation completed.", name().c_str());
        
        // 输出最终到达的位置
        Position3D final_pos;
        final_pos.x = fcu_odom_ptr->pose.pose.position.x;
        final_pos.y = fcu_odom_ptr->pose.pose.position.y;
        final_pos.z = fcu_odom_ptr->pose.pose.position.z;
        setOutput("final_position", final_pos);
        
        ROS_INFO("[%s]: Final position: (%.2f, %.2f, %.2f)", 
                 name().c_str(), final_pos.x, final_pos.y, final_pos.z);
        
        sendHoverCommand();
        return NodeStatus::SUCCESS;
    }

    if (!has_planner_cmd) return NodeStatus::RUNNING;

    // Ego-planner 输出经过 publishPosition 做 ENU→NED 转换
    geometry_msgs::Point ego_pos = last_planner_cmd.position;
    double ego_yaw = control_yaw ? last_planner_cmd.yaw : initial_yaw_;
    
    publishPosition(ego_pos, ego_yaw);
    
    return NodeStatus::RUNNING;
}

BT_REGISTER_NODES(factory) {
    factory.registerNodeType<PlanningControl>("PlanningControl");
}
