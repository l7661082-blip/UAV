/******************************************************************************
 * @file       aruco_detector.h
 * @brief      ArUco 二维码检测行为树节点（通过服务控制独立检测节点）
 *
 * @details    不直接处理图像，通过 /aruco/enable 服务开关控制
 *             aruco_detect_node 进行检测，从 /aruco/detected_id 读取结果
 *
 * @author     郭定<1604289397@qq.com>
 * @date       2026-03-16
 ******************************************************************************/
#ifndef ARUCO_DETECTOR_H
#define ARUCO_DETECTOR_H

#include "base/drone_action_node.h"
#include <std_msgs/Int32.h>
#include <std_srvs/SetBool.h>

class ArucoDetector : public DroneActionNode {
public:
    ArucoDetector(const std::string& name, const NodeConfig& config);
    
    static PortsList providedPorts();

    virtual NodeStatus onStart() override;
    virtual NodeStatus onRunning() override;
    virtual void onHalted() override;

private:
    // 结果回调
    void idCallback(const std_msgs::Int32::ConstPtr& msg);
    
    // 开关控制
    bool setDetectionEnabled(bool enabled);
    
    // 订阅器 & 客户端
    ros::Subscriber id_sub_;
    ros::ServiceClient enable_client_;
    
    // 状态
    bool has_result_;
    int detected_id_;
    ros::Time start_time_;
    
    // 参数
    double timeout_;
};

#endif // ARUCO_DETECTOR_H
