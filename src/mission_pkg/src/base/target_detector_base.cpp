/******************************************************************************
 * @file       target_detector_base.cpp
 * @brief      目标检测基类实现
 *
 * @details    只负责 YOLO 订阅和基本过滤，将处理逻辑委托给子类
 *
 * @author     郭定<1604289397@qq.com>
 * @date       2026-01-13
 *****************************************************************************/
#include "base/target_detector_base.h"

TargetDetectorBase::TargetDetectorBase(const std::string& name, const NodeConfig& config)
    : DroneActionNode(name, config), has_new_objects_msg_(false) {
    
    auto nh = mct->getROSHandle();
    std::string det_topic;
    nh->param<std::string>("detect_result_topic", det_topic, "/objects");
    objs_sub = nh->subscribe(det_topic, 1, &TargetDetectorBase::objsCallback, this);
}

PortsList TargetDetectorBase::baseProvidedPorts() {
    return { InputPort<std::string>("object_name", "需要检测的目标类别名称"),
             InputPort<double>("confidence_threshold", 0.9, "检测置信度阈值(0-1)") };
}

void TargetDetectorBase::objsCallback(const common_msgs::Objects::ConstPtr& msg) {
    last_objs_ = *msg;
    has_new_objects_msg_ = true;
    // ROS_INFO("11");
}

NodeStatus TargetDetectorBase::onStart() {
    auto res = getInput<std::string>("object_name");
    if (!res) throw RuntimeError("Missing object_name");
    target_class = res.value();

    auto conf_res = getInput<double>("confidence_threshold");
    confidence_threshold = conf_res.value();

    has_new_objects_msg_ = false;
    
    ROS_INFO("[%s]: Detecting target: %s, conf_th: %.2f", 
             name().c_str(), target_class.c_str(), confidence_threshold);

    return NodeStatus::RUNNING;
}

NodeStatus TargetDetectorBase::onRunning() {
    // 1. 检查是否有新的感知消息
    if (!has_new_objects_msg_) {
        return handleNoDetection();
    }
    
    has_new_objects_msg_ = false;  // 标记已消费

    // 2. 在消息中查找特定目标
    common_msgs::Obj target_obj;
    if (findTargetInObjects(last_objs_, target_obj)) {
        // 3. 找到目标，委托子类处理

        return handleDetectedObject(target_obj);
    }
    
    // 4. 收到了消息但没找到目标
    return handleNoDetection();
}

bool TargetDetectorBase::findTargetInObjects(const common_msgs::Objects& msg, common_msgs::Obj& out_obj) {
    for (const auto& obj : msg.objects) {
        if (obj.class_name == target_class && obj.score >= confidence_threshold) {
            out_obj = obj;
            return true;
        }
    }
    return false;
}
