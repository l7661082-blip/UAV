/******************************************************************************
 * @file       target_tracker.cpp
 * @brief      TargetTracker 节点实现
 *
 * @details    简单像素跟踪：直接提取像素坐标+简单平滑
 *
 * @author     郭定<1604289397@qq.com>
 * @date       2026-01-13
 *****************************************************************************/
#include "nodes/perception/target_tracker.h"

TargetTracker::TargetTracker(const std::string& name, const NodeConfig& config)
    : TargetDetectorBase(name, config) {}

PortsList TargetTracker::providedPorts() {
    auto ports = baseProvidedPorts();
    ports.insert(InputPort<int>("filter_window_size", 10, "像素坐标平滑窗口大小"));
    ports.insert(OutputPort<PixelTarget>("pixel_target", "像素目标（中心+尺寸）"));
    return ports;
}

NodeStatus TargetTracker::onStart() {
    // 调用基类 onStart
    NodeStatus base_status = TargetDetectorBase::onStart();
    
    // 获取自己的参数
    auto window_res = getInput<int>("filter_window_size");
    filter_window_size = window_res.value();

    pixel_history.clear();
    
    ROS_INFO("[%s]: Pixel tracking filter window: %d", name().c_str(), filter_window_size);
    
    return base_status;
}

NodeStatus TargetTracker::handleDetectedObject(const common_msgs::Obj& obj) {
    // 直接提取像素坐标
    double pixel_width = obj.right_bottom_x - obj.left_top_x;
    double pixel_height = obj.right_bottom_y - obj.left_top_y;
    
    if (pixel_width <= 0 || pixel_height <= 0) {
        return handleNoDetection();  // 无效尺寸视为未检测到
    }
    
    // 添加到历史
    PixelTarget current_pixel(
        obj.center_x,
        obj.center_y,
        pixel_width,
        pixel_height,
        ros::Time::now().toSec(),
        true // ✅ is_detected = true
    );
    
    pixel_history.push_back(current_pixel);
    while (static_cast<int>(pixel_history.size()) > filter_window_size) {
        pixel_history.pop_front();
    }
    
    // 输出平滑后的像素坐标
    PixelTarget smoothed = getSmoothedPixel();
    smoothed.timestamp = ros::Time::now().toSec(); // 更新时间戳
    smoothed.is_detected = true;                   // 再次确认标志
    setOutput("pixel_target", smoothed);
    
    ROS_INFO_THROTTLE(0.5, "[%s]: Tracking at pixel (%.1f, %.1f)", 
                     name().c_str(), smoothed.center_x, smoothed.center_y);
    
    return NodeStatus::RUNNING;
}

NodeStatus TargetTracker::handleNoDetection() {
    // 即使未检测到，也发布数据，但标记 is_detected = false
    // 保留旧的平滑坐标（或者清零，视需求而定，这里选择发送空/旧数据+false标志）
    PixelTarget empty_target;
    empty_target.timestamp = ros::Time::now().toSec();
    empty_target.is_detected = false; // ❌ is_detected = false
    
    setOutput("pixel_target", empty_target);
    
    // 清空历史，避免"鬼影"（突然出现时用到旧历史平均）
    // 或者不清空以保持连续性？建议清空，因为断开了
    pixel_history.clear();
    
    ROS_WARN_THROTTLE(1.0, "[%s]: No target detected, publishing INVALID target", name().c_str());
    return NodeStatus::RUNNING;
}

PixelTarget TargetTracker::getSmoothedPixel() {
    if (pixel_history.empty()) {
        return PixelTarget(0, 0, 0, 0, ros::Time::now().toSec());
    }
    
    // 简单移动平均
    double sum_x = 0, sum_y = 0, sum_w = 0, sum_h = 0;
    for (const auto& px : pixel_history) {
        sum_x += px.center_x;
        sum_y += px.center_y;
        sum_w += px.width;
        sum_h += px.height;
    }
    
    int n = pixel_history.size();
    return PixelTarget(
        sum_x / n,
        sum_y / n,
        sum_w / n,
        sum_h / n,
        ros::Time::now().toSec()
    );
}

BT_REGISTER_NODES(factory) {
    factory.registerNodeType<TargetTracker>("TargetTracker");
}
