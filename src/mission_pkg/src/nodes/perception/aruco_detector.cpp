/******************************************************************************
 * @file       aruco_detector.cpp
 * @brief      ArUco 二维码检测行为树节点实现（服务开关版）
 *
 * @details    通过 /aruco/enable 服务控制独立检测节点 aruco_detect_node
 *             onStart() 开启检测 → onRunning() 等待结果 → 完成/中止时关闭
 *
 * @author     郭定<1604289397@qq.com>
 * @date       2026-03-16
 ******************************************************************************/
#include "nodes/perception/aruco_detector.h"

ArucoDetector::ArucoDetector(const std::string& name, const NodeConfig& config)
    : DroneActionNode(name, config), has_result_(false), detected_id_(-1) {
    
    auto nh = mct->getROSHandle();
    
    // 订阅检测结果
    id_sub_ = nh->subscribe("/aruco/detected_id", 10, &ArucoDetector::idCallback, this);
    
    // 服务客户端
    enable_client_ = nh->serviceClient<std_srvs::SetBool>("/aruco/enable");
}

PortsList ArucoDetector::providedPorts() {
    return { InputPort<double>("timeout", 20.0, "检测超时时间(秒)"),
             OutputPort<int>("aruco_id", "检测到的 ArUco ID") };
}

void ArucoDetector::idCallback(const std_msgs::Int32::ConstPtr& msg) {
    detected_id_ = msg->data;
    has_result_ = true;
    ROS_INFO("[%s]: 收到 ArUco ID: %d", name().c_str(), detected_id_);
}

bool ArucoDetector::setDetectionEnabled(bool enabled) {
    std_srvs::SetBool srv;
    srv.request.data = enabled;
    
    if (enable_client_.call(srv)) {
        ROS_INFO("[%s]: %s ArUco 检测 -> %s", 
                 name().c_str(), 
                 enabled ? "开启" : "关闭",
                 srv.response.success ? "成功" : "失败");
        return srv.response.success;
    } else {
        ROS_ERROR("[%s]: 调用 /aruco/enable 服务失败 (节点是否启动?)", name().c_str());
        return false;
    }
}

NodeStatus ArucoDetector::onStart() {
    auto timeout_res = getInput<double>("timeout");
    timeout_ = timeout_res.value();
    
    has_result_ = false;
    detected_id_ = -1;
    start_time_ = ros::Time::now();
    
    // 开启检测
    if (!setDetectionEnabled(true)) {
        ROS_ERROR("[%s]: 无法开启 ArUco 检测，请确认 aruco_detect_node 已启动", name().c_str());
        return NodeStatus::FAILURE;
    }
    
    ROS_INFO("[%s]: ArUco 检测已开启，等待结果... (超时: %.1fs)", name().c_str(), timeout_);
    return NodeStatus::RUNNING;
}

NodeStatus ArucoDetector::onRunning() {
    // 检查超时
    double elapsed = (ros::Time::now() - start_time_).toSec();
    if (elapsed > timeout_) {
        ROS_WARN("[%s]: ArUco 检测超时 (%.1fs)", name().c_str(), timeout_);
        setDetectionEnabled(false);
        return NodeStatus::FAILURE;
    }
    
    // 检查是否收到结果
    if (has_result_) {
        ROS_INFO("[%s]: ================================", name().c_str());
        ROS_INFO("[%s]:   ARUCO ID DETECTED: %d", name().c_str(), detected_id_);
        ROS_INFO("[%s]: ================================", name().c_str());
        
        // 输出到黑板
        setOutput("aruco_id", detected_id_);
        
        // 关闭检测
        setDetectionEnabled(false);
        
        return NodeStatus::SUCCESS;
    }
    
    // 悬停等待
    sendHoverCommand();
    
    ROS_INFO_THROTTLE(2.0, "[%s]: 等待 ArUco 检测结果... (%.1f/%.1fs)", 
                     name().c_str(), elapsed, timeout_);
    return NodeStatus::RUNNING;
}

void ArucoDetector::onHalted() {
    // 被中止时关闭检测
    setDetectionEnabled(false);
    DroneActionNode::onHalted();
}

BT_REGISTER_NODES(factory) {
    factory.registerNodeType<ArucoDetector>("ArucoDetector");
}
