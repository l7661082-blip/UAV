/******************************************************************************
 * @file       land.cpp
 * @brief      Land 行为树节点 - 无人机降落控制
 *
 * @author     郭定<1604289397@qq.com>
 * @date       2026-01-06
 ******************************************************************************/
#include "nodes/action/land.h"

Land::Land(const std::string& name, const NodeConfig& config)
    : DroneActionNode(name, config) {}

PortsList Land::providedPorts() {
    return { InputPort<int>("use_speed", 0, "0: AUTO.LAND, 1: 速度下降") };
}

NodeStatus Land::onStart() {
    auto res = getInput<int>("use_speed");
    use_speed_mode = (res.value() != 0);
    return NodeStatus::RUNNING;
}

NodeStatus Land::onRunning() {
    if (use_speed_mode) {
        publishVelocityWithYawRate(0, 0, -0.3, 0); // 0.3m/s 下降，yaw_rate=0保持航向
    } else {
        throttledSetMode("AUTO.LAND");
    }

    if (!fcu_state_ptr->armed) {
        ROS_INFO("[%s]: Landed and disarmed.", name().c_str());
        return NodeStatus::SUCCESS;
    }

    return NodeStatus::RUNNING;
}

BT_REGISTER_NODES(factory) {
    factory.registerNodeType<Land>("Land");
}
