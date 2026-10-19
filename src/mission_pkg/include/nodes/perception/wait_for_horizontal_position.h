/******************************************************************************
 * @file       wait_for_horizontal_position.h
 * @brief      WaitForHorizontalPosition 节点头文件
 *
 * @details    等待目标出现在画面的合适水平位置后返回 SUCCESS
 *             用于确保目标在特定的 x 坐标范围内（如中间偏右）
 *
 * @author     郭定<1604289397@qq.com>
 * @date       2026-01-22
 *****************************************************************************/
#ifndef WAIT_FOR_HORIZONTAL_POSITION_H
#define WAIT_FOR_HORIZONTAL_POSITION_H

#include "base/target_detector_base.h"

/**
 * @brief 等待目标出现在合适的水平位置
 * 
 * 检测目标的像素坐标 x 是否在指定的水平范围内。
 * 适用于判断目标是否进入起始区等场景。
 * 
 * 输入端口：
 *   - object_name: 目标类别名称
 *   - confidence_threshold: 置信度阈值
 *   - x_min_ratio: 最小 x 位置比例 (0.0-1.0)，默认 0.4（画面 40% 处）
 *   - x_max_ratio: 最大 x 位置比例 (0.0-1.0)，默认 0.8（画面 80% 处）
 *   - image_width: 图像宽度，默认 640
 *   - timeout: 超时时间（秒），默认 120
 * 
 * 返回状态：
 *   - SUCCESS: 目标出现在合适的水平位置
 *   - RUNNING: 目标未出现或不在合适位置
 *   - FAILURE: 超时
 */
class WaitForHorizontalPosition : public TargetDetectorBase {
public:
    WaitForHorizontalPosition(const std::string& name, const BT::NodeConfig& config);
    
    static BT::PortsList providedPorts();
    
    BT::NodeStatus onStart() override;

protected:
    BT::NodeStatus handleDetectedObject(const common_msgs::Obj& obj) override;
    BT::NodeStatus handleNoDetection() override;
    
private:
    // 判断目标是否在合适的水平位置
    bool isInValidXRange(double center_x);
    
    // 参数
    double x_min_ratio_;       // 最小 x 位置比例
    double x_max_ratio_;       // 最大 x 位置比例
    int image_width_;          // 图像宽度
    double timeout_;           // 超时时间（秒）
    
    // 状态
    ros::Time start_time_;     // 开始时间
};

#endif // WAIT_FOR_HORIZONTAL_POSITION_H
