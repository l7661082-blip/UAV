/******************************************************************************
 * @file       target_detector_base.h
 * @brief      目标检测基类 - 最小化设计
 *
 * @details    只负责 YOLO 订阅、类别过滤、置信度筛选
 *             子类决定如何处理检测到的对象
 *
 * @author     郭定<1604289397@qq.com>
 * @date       2026-01-13
 *****************************************************************************/
#ifndef TARGET_DETECTOR_BASE_H
#define TARGET_DETECTOR_BASE_H

#include "base/drone_action_node.h"
#include <common_msgs/Objects.h>
#include <mutex>

/**
 * @brief 目标检测基类（最小化设计）
 * 
 * 职责：
 * 1. 订阅 YOLO 检测结果
 * 2. 按类别和置信度过滤
 * 3. 调用子类的 handleDetectedObject 处理匹配对象
 * 
 * 子类职责：
 * - 实现 handleDetectedObject 定义如何处理检测结果
 * - 定义自己的输出端口和处理逻辑
 */
class TargetDetectorBase : public DroneActionNode {
public:
    TargetDetectorBase(const std::string& name, const NodeConfig& config);
    
    NodeStatus onStart() override;
    NodeStatus onRunning() override;

protected:
    /**
     * @brief 处理检测到的对象（由子类实现）
     * @param obj YOLO 检测到的对象
     * @return 节点状态（SUCCESS/RUNNING/FAILURE）
     */
    virtual NodeStatus handleDetectedObject(const common_msgs::Obj& obj) = 0;
    
    // 子类可选实现：如何处理未检测到目标的情况（默认: RUNNING）
    virtual NodeStatus handleNoDetection() { return NodeStatus::RUNNING; }
    
    // 子类可选实现：如何处理没有新数据的情况（默认: RUNNING）
    virtual NodeStatus handleNoNewData() { return NodeStatus::RUNNING; }
    
    /**
     * @brief 提供基类公共端口
     */
    static PortsList baseProvidedPorts();
    
    // 通用参数
    std::string target_class;
    double confidence_threshold;


private:
    void objsCallback(const common_msgs::Objects::ConstPtr& msg);
    
    // 辅助函数：在对象列表中查找符合条件的目标
    bool findTargetInObjects(const common_msgs::Objects& msg, common_msgs::Obj& out_obj);
    
    // YOLO 订阅
    ros::Subscriber objs_sub;
    common_msgs::Objects last_objs_;
    bool has_new_objects_msg_; // 明确标识是否收到新的物体消息
    ros::Time last_objects_received_;
    double detection_timeout_;
    bool stale_reported_;
    std::mutex objects_mutex_;
};

#endif // TARGET_DETECTOR_BASE_H
