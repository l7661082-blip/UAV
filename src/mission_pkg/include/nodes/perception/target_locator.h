/******************************************************************************
 * @file       target_locator.h
 * @brief      TargetLocator 节点 - 完整 3D 目标定位
 *
 * @details    检测目标并输出 3D 坐标后立即 SUCCESS，用于 PlanningControl
 *             包含：物理尺寸估算、遮挡检测与补偿、3D 坐标转换、位置验证、历史平滑
 *
 * @author     郭定<1604289397@qq.com>
 * @date       2026-01-13
 *****************************************************************************/
#ifndef TARGET_LOCATOR_H
#define TARGET_LOCATOR_H

#include "base/target_detector_base.h"
#include "utils/pose_utils.h"
#include <deque>

class TargetLocator : public TargetDetectorBase {
public:
    TargetLocator(const std::string& name, const NodeConfig& config);
    
    static PortsList providedPorts();
    
    NodeStatus onStart() override;

protected:
    NodeStatus handleDetectedObject(const common_msgs::Obj& obj) override;

private:
    // 辅助函数
    bool isPartiallyOutOfView(const common_msgs::Obj& obj);
    bool isWithinExpectedRange(const Eigen::Vector3d& pos);
    void addToHistory(const Eigen::Vector3d& pos);
    Eigen::Vector3d getSmoothedPosition();
    
    // 3D 定位参数
    double physical_width;
    double physical_height;
    double expected_aspect_ratio;
    std::string occlusion_direction;
    Position3D expected_pos;
    double xy_tolerance, z_tolerance;
    int filter_window_size;
    
    // 相机参数
    double fx, fy, cx, cy;
    Eigen::Matrix3d cam_R_body;
    Eigen::Vector3d cam_T_body;
    double image_width, image_height;
    
    // 历史平滑
    std::deque<Eigen::Vector3d> position_history;
    Eigen::Vector3d best_estimate;
};

#endif // TARGET_LOCATOR_H
