/******************************************************************************
 * @file       wait_for_horizontal_position.cpp
 * @brief      WaitForHorizontalPosition 节点实现
 *
 * @details    等待目标出现在画面的合适水平位置后返回 SUCCESS
 *             用于确保目标在特定的 x 坐标范围内（如中间偏右）
 *
 * @author     郭定<1604289397@qq.com>
 * @date       2026-01-22
 *****************************************************************************/
#include "nodes/perception/wait_for_horizontal_position.h"

WaitForHorizontalPosition::WaitForHorizontalPosition(const std::string& name, const NodeConfig& config)
    : TargetDetectorBase(name, config) {}

PortsList WaitForHorizontalPosition::providedPorts() {
    auto ports = baseProvidedPorts();
    ports.insert(InputPort<double>("x_min_ratio", 0.4, "最小 x 位置比例 (0.0-1.0)"));
    ports.insert(InputPort<double>("x_max_ratio", 0.8, "最大 x 位置比例 (0.0-1.0)"));
    ports.insert(InputPort<int>("image_width", 640, "图像宽度"));
    ports.insert(InputPort<double>("timeout", 120.0, "超时时间（秒）"));
    return ports;
}

NodeStatus WaitForHorizontalPosition::onStart() {
    // 调用基类 onStart
    NodeStatus base_status = TargetDetectorBase::onStart();
    
    // 获取参数
    auto x_min_res = getInput<double>("x_min_ratio");
    x_min_ratio_ = x_min_res.value();
    
    auto x_max_res = getInput<double>("x_max_ratio");
    x_max_ratio_ = x_max_res.value();
    
    auto width_res = getInput<int>("image_width");
    image_width_ = width_res.value();
    
    auto timeout_res = getInput<double>("timeout");
    timeout_ = timeout_res.value();
    
    // 初始化状态
    start_time_ = ros::Time::now();
    
    // 计算像素范围
    double x_min_pixel = x_min_ratio_ * image_width_;
    double x_max_pixel = x_max_ratio_ * image_width_;
    
    ROS_INFO("[%s]: Waiting for '%s' in horizontal range [%.0f, %.0f] pixels (%.0f%% - %.0f%%)", 
             name().c_str(), target_class.c_str(), 
             x_min_pixel, x_max_pixel,
             x_min_ratio_ * 100, x_max_ratio_ * 100);
    ROS_INFO("[%s]: Timeout=%.1fs",
             name().c_str(), timeout_);
    
    return base_status;
}

NodeStatus WaitForHorizontalPosition::handleDetectedObject(const common_msgs::Obj& obj) {
    // 检查是否超时
    double elapsed = (ros::Time::now() - start_time_).toSec();
    if (elapsed > timeout_) {
        ROS_ERROR("[%s]: Timeout! Target not found in valid horizontal position within %.1fs", 
                  name().c_str(), timeout_);
        return NodeStatus::FAILURE;
    }
    
    // 检查目标是否在合适的水平位置
    if (isInValidXRange(obj.center_x)) {
        double x_ratio = static_cast<double>(obj.center_x) / image_width_;
        ROS_INFO("[%s]: ✓ Target in valid position! x=%u (%.1f%%) - Proceeding!",
                 name().c_str(), obj.center_x, x_ratio * 100);
        return NodeStatus::SUCCESS;
    } else {
        double x_ratio = static_cast<double>(obj.center_x) / image_width_;
        ROS_INFO_THROTTLE(1.0, "[%s]: Target at x=%u (%.1f%%) - outside valid range [%.0f%%, %.0f%%], waiting...",
                         name().c_str(), obj.center_x, x_ratio * 100,
                         x_min_ratio_ * 100, x_max_ratio_ * 100);
    }
    
    return NodeStatus::RUNNING;
}

NodeStatus WaitForHorizontalPosition::handleNoDetection() {
    // 检查是否超时
    double elapsed = (ros::Time::now() - start_time_).toSec();
    if (elapsed > timeout_) {
        ROS_ERROR("[%s]: Timeout! Target not detected within %.1fs", 
                  name().c_str(), timeout_);
        return NodeStatus::FAILURE;
    }
    
    ROS_INFO_THROTTLE(2.0, "[%s]: Waiting for target... (%.1fs / %.1fs)",
                     name().c_str(), elapsed, timeout_);
    
    return NodeStatus::RUNNING;
}

bool WaitForHorizontalPosition::isInValidXRange(double center_x) {
    // 计算有效范围
    double x_min = x_min_ratio_ * image_width_;
    double x_max = x_max_ratio_ * image_width_;
    
    return (center_x >= x_min && center_x <= x_max);
}

BT_REGISTER_NODES(factory) {
    factory.registerNodeType<WaitForHorizontalPosition>("WaitForHorizontalPosition");
}
