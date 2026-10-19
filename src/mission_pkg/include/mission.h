/******************************************************************************
 * @file       drone_action_node.h
 * @brief      行为树节点基类 - 封装无人机控制的通用功能
 *
 * @details    为所有无人机行为树节点提供统一的状态访问、控制指令发布等
 *
 * @author     郭定<1604289397@qq.com>
 * @date       2026-01-09
 ******************************************************************************/
#ifndef MISSION_H
#define MISSION_H

#include <utils/common.hpp>

using namespace BT;

class Mission : public SyncActionNode
{
public:
  Mission(const std::string &name, const NodeConfig &config);

  static PortsList providedPorts();

  NodeStatus tick() override;
};

#endif  // MISSION_H
