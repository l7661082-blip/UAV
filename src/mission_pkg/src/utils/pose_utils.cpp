/******************************************************************************
 * @file       pose_utils.cpp
 * @brief      位姿坐标转换工具类实现
 *
 * @author     郭定<1604289397@qq.com>
 * @date       2026-01-06
 ******************************************************************************/
#include "utils/pose_utils.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

Eigen::Vector3d PoseUtils::cameraToWorld(const Eigen::Vector3d& cam_point, 
                                        const geometry_msgs::Pose& fcu_pose,
                                        const Eigen::Matrix3d& cam_to_body_R,
                                        const Eigen::Vector3d& cam_to_body_T) {
    // 1. 相机系 -> 机身系
    Eigen::Vector3d p_body = cam_to_body_R * cam_point + cam_to_body_T;

    // 2. 机身系 -> 世界系
    return bodyToWorld(p_body, fcu_pose);
}

double PoseUtils::getYawFromQuat(const geometry_msgs::Quaternion& q_msg) {
    tf2::Quaternion q;
    tf2::fromMsg(q_msg, q);
    double r, p, y;
    tf2::Matrix3x3(q).getRPY(r, p, y);
    return y;
}

Eigen::Vector3d PoseUtils::bodyToWorld(const Eigen::Vector3d& body_point,
                                      const geometry_msgs::Pose& fcu_pose) {
    Eigen::Quaterniond q(fcu_pose.orientation.w, fcu_pose.orientation.x, 
                         fcu_pose.orientation.y, fcu_pose.orientation.z);
    Eigen::Vector3d t(fcu_pose.position.x, fcu_pose.position.y, fcu_pose.position.z);
    
    return q * body_point + t;
}
