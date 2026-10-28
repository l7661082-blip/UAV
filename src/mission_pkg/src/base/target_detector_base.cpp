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
    : DroneActionNode(name, config), has_new_objects_msg_(false),
      detection_timeout_(1.0), stale_reported_(false) {
    
    auto nh = mct->getROSHandle();
    std::string det_topic;
    nh->param<std::string>("detect_result_topic", det_topic, "/objects");
    nh->param<double>("detection_timeout", detection_timeout_, 1.0);
    objs_sub = nh->subscribe(det_topic, 1, &TargetDetectorBase::objsCallback, this);
}

PortsList TargetDetectorBase::baseProvidedPorts() {
    return { InputPort<std::string>("object_name", "需要检测的目标类别名称"),
             InputPort<double>("confidence_threshold", 0.9, "检测置信度阈值(0-1)") };
}

void TargetDetectorBase::objsCallback(const common_msgs::Objects::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(objects_mutex_);
    last_objs_ = *msg;
    last_objects_received_ = ros::Time::now();
    has_new_objects_msg_ = true;
    stale_reported_ = false;
    // ROS_INFO("11");
}

NodeStatus TargetDetectorBase::onStart() {
    auto res = getInput<std::string>("object_name");
    if (!res) throw RuntimeError("Missing object_name");
    target_class = res.value();

    auto conf_res = getInput<double>("confidence_threshold");
    confidence_threshold = conf_res.value();

    {
        std::lock_guard<std::mutex> lock(objects_mutex_);
        has_new_objects_msg_ = false;
        stale_reported_ = false;
        last_objects_received_ = ros::Time::now();
    }
    
    ROS_INFO("[%s]: Detecting target: %s, conf_th: %.2f", 
             name().c_str(), target_class.c_str(), confidence_threshold);

    return NodeStatus::RUNNING;
}

NodeStatus TargetDetectorBase::onRunning() {
    common_msgs::Objects objects;
    bool has_new = false;
    bool became_stale = false;
    {
        std::lock_guard<std::mutex> lock(objects_mutex_);
        has_new = has_new_objects_msg_;
        if (has_new) {
            objects = last_objs_;
            has_new_objects_msg_ = false;
        } else if (!stale_reported_ && detection_timeout_ > 0.0 &&
                   (ros::Time::now() - last_objects_received_).toSec() > detection_timeout_) {
            stale_reported_ = true;
            became_stale = true;
        }
    }
    if (!has_new) {
        // 控制循环可能快于图像帧率；只在消息真正过期时报告一次丢失。
        if (became_stale) return handleNoDetection();
        return handleNoNewData();
    }

    // 2. 在消息中查找特定目标
    common_msgs::Obj target_obj;
    if (findTargetInObjects(objects, target_obj)) {
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
