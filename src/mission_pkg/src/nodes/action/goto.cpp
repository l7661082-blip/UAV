/******************************************************************************
 * @file       goto.cpp
 * @brief      GoTo 行为树节点 - 飞往指定位置
 *
 * @author     郭定<1604289397@qq.com>
 * @date       2026-01-06
 ******************************************************************************/
#include "nodes/action/goto.h"
#include "utils/pose_utils.h"
#include "utils/flight_boundary.h"

GoTo::GoTo(const std::string& name, const NodeConfig& config)
    : DroneActionNode(name, config) {}

PortsList GoTo::providedPorts() {
    return { InputPort<Position3D>("goal", "目标位置 (x, y, z), 为空则保持当前位置"),
             InputPort<double>("yaw", "目标绝对航向 (rad), 为空则保持当前航向"),
             InputPort<int>("frame", 0, "坐标系: 0=世界系(默认), 1=机体系"),
             InputPort<bool>("allow_outside_flight_boundary", false, "仅场外起降区返回使用"),
             InputPort<Position3D>("clearance_from", "框平面中心，机体系前进前检查"),
             InputPort<double>("min_frame_clearance", 0.0, "沿航向离开框平面的最小距离"),
             InputPort<double>("tolerance", "到达容差"),
             InputPort<double>("yaw_tolerance", "偏航到达容差(rad)"),
             InputPort<double>("yaw_rate", 0.5, "最大偏航角速度 (rad/s)") };
}

NodeStatus GoTo::onStart() {
    // 获取当前位置和航向
    geometry_msgs::Point current_pos = fcu_odom_ptr->pose.pose.position;
    double current_yaw = PoseUtils::getYawFromQuat(fcu_odom_ptr->pose.pose.orientation);

    // 解析目标位置
    auto goal_res = getInput<Position3D>("goal");
    auto frame_res = getInput<int>("frame");
    int frame = frame_res.value();

    const double min_clearance = getInput<double>("min_frame_clearance").value();
    if (!std::isfinite(min_clearance) || min_clearance < 0.0) {
        ROS_ERROR("[%s] Invalid minimum frame clearance", name().c_str());
        sendHoverCommand();
        return NodeStatus::FAILURE;
    }
    if (min_clearance > 0.0) {
        auto center = getInput<Position3D>("clearance_from");
        if (frame != 1 || !center || !std::isfinite(center.value().x)
                || !std::isfinite(center.value().y)) {
            ROS_ERROR("[%s] Frame clearance requires body-frame GoTo and frame center", name().c_str());
            sendHoverCommand();
            return NodeStatus::FAILURE;
        }
        // 反向穿框后，应位于当前航向所指一侧；不在净空侧就拒绝前进。
        const double signed_clearance =
            (current_pos.x - center.value().x) * std::cos(current_yaw) +
            (current_pos.y - center.value().y) * std::sin(current_yaw);
        if (signed_clearance < min_clearance) {
            ROS_WARN("[%s] Frame clearance %.3fm < %.3fm; body move blocked",
                     name().c_str(), signed_clearance, min_clearance);
            sendHoverCommand();
            return NodeStatus::FAILURE;
        }
    }

    if (goal_res) {
        Position3D goal = goal_res.value();
        if (frame == 1) {  // 1 = 机体系
            // 机体坐标系转世界坐标系：
            // goal.x = 机体前进方向
            // goal.y = 机体左移方向
            // 需要根据当前yaw角度旋转到世界坐标系
            target_pos.x = current_pos.x + goal.x * std::cos(current_yaw) - goal.y * std::sin(current_yaw);
            target_pos.y = current_pos.y + goal.x * std::sin(current_yaw) + goal.y * std::cos(current_yaw);
            target_pos.z = current_pos.z + goal.z;
        } else {  // 0 = 世界系(默认)
            target_pos.x = goal.x;
            target_pos.y = goal.y;
            target_pos.z = goal.z;
        }
    } else {
        target_pos = current_pos;
    }

    const bool allow_outside_flight_boundary = getInput<bool>("allow_outside_flight_boundary").value();
    if (allow_outside_flight_boundary) {
        if (!std::isfinite(target_pos.x) || !std::isfinite(target_pos.y)
                || !std::isfinite(target_pos.z)) {
            ROS_ERROR("[%s] Non-finite landing-area goal rejected", name().c_str());
            sendHoverCommand();
            return NodeStatus::FAILURE;
        }
        ROS_WARN("[%s] Explicit landing-area exception: navigation goal is not clamped", name().c_str());
    } else {
        FlightBoundary boundary;
        if (!FlightBoundary::load(boundary)) {
            sendHoverCommand();
            return NodeStatus::FAILURE;
        }
        if (!boundary.clamp(target_pos, name())) {
            sendHoverCommand();
            return NodeStatus::FAILURE;
        }
    }

    // 解析航向
    auto yaw_res = getInput<double>("yaw");
    if (yaw_res) {
        target_yaw = yaw_res.value();
        check_yaw = true;
    } else {
        target_yaw = current_yaw;
        check_yaw = false;
    }

    auto tol_res = getInput<double>("tolerance");
    if (tol_res) {
        tolerance = tol_res.value();
    } else {
        ros::NodeHandle nh;
        nh.param("tolerance", tolerance, 0.2);
    }
    
    auto yaw_tol_res = getInput<double>("yaw_tolerance");
    if (yaw_tol_res) {
        yaw_tolerance = yaw_tol_res.value();
    } else {
        ros::NodeHandle nh;
        nh.param("yaw_tolerance", yaw_tolerance, 0.15);
    }

    auto yaw_rate_res = getInput<double>("yaw_rate");
    yaw_rate_limit = yaw_rate_res.value();

    // 初始化偏航设定点为当前实际航向
    current_yaw_sp = current_yaw;
    last_loop_time = ros::Time::now();

    ROS_INFO("[%s]: GoTo start -> target: (%.2f, %.2f, %.2f), yaw: %.2f, frame: %d, tol: %.2f, yaw_tol: %.2f, yaw_rate: %.2f", 
             name().c_str(), target_pos.x, target_pos.y, target_pos.z, target_yaw, frame, tolerance, yaw_tolerance, yaw_rate_limit);
    return NodeStatus::RUNNING;
}

NodeStatus GoTo::onRunning() {
    // 计算时间差
    ros::Time now = ros::Time::now();
    double dt = (now - last_loop_time).toSec();
    last_loop_time = now;

    // 偏航角平滑插值 (Ramp)
    double yaw_error = target_yaw - current_yaw_sp;
    
    // 归一化误差到 [-PI, PI]
    while (yaw_error > M_PI) yaw_error -= 2 * M_PI;
    while (yaw_error < -M_PI) yaw_error += 2 * M_PI;

    double max_step = yaw_rate_limit * dt;

    if (std::abs(yaw_error) <= max_step) {
        current_yaw_sp = target_yaw;
    } else {
        // 向目标方向步进
        current_yaw_sp += (yaw_error > 0 ? 1.0 : -1.0) * max_step;
    }

    // 归一化设定点 (防止长期累积溢出，虽然 cos/sin 无所谓，但保持规范)
    while (current_yaw_sp > M_PI) current_yaw_sp -= 2 * M_PI;
    while (current_yaw_sp < -M_PI) current_yaw_sp += 2 * M_PI;

    // 发布位置指令 (使用平滑后的 current_yaw_sp)
    publishPosition(target_pos, current_yaw_sp);

    // 到达判断 (只有在明确传入了yaw参数时，才检查yaw是否到达)
    double arrival_yaw = check_yaw ? target_yaw : std::numeric_limits<double>::quiet_NaN();
    if (checkArrival(target_pos, tolerance, arrival_yaw, yaw_tolerance)) {
        ROS_INFO("[%s]: GoTo completed (pos & yaw).", name().c_str());
        sendHoverCommand();
        return NodeStatus::SUCCESS;
    }

    return NodeStatus::RUNNING;
}

BT_REGISTER_NODES(factory) {
    factory.registerNodeType<GoTo>("GoTo");
}
