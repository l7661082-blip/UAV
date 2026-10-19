/******************************************************************************
 * @file       land.h
 * @brief      Land 行为树节点声明
 *
 * @author     郭定<1604289397@qq.com>
 * @date       2026-01-06
 ******************************************************************************/
#ifndef LAND_H
#define LAND_H

#include "base/drone_action_node.h"

class Land : public DroneActionNode {
public:
    Land(const std::string& name, const NodeConfig& config);
    
    static PortsList providedPorts();

    virtual NodeStatus onStart() override;
    virtual NodeStatus onRunning() override;

private:
    bool use_speed_mode;
};

#endif // LAND_H
