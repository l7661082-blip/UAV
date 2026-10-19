/******************************************************************************
 * @file       pose_utils.h
 * @brief      位姿坐标转换工具类
 *
 * @details    提供相机坐标系、机体坐标系、世界坐标系之间的转换
 *
 * @author     郭定<1604289397@qq.com>
 * @date       2026-01-06
 ******************************************************************************/
#ifndef POSE_UTILS_H
#define POSE_UTILS_H

#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

class PoseUtils {
public:
    /**
     * @brief 将相机坐标系下的点转换到世界坐标系 (local_origin)
     * @param cam_point 相机坐标系下的 3D 点 (x: 右, y: 下, z: 前)
     * @param fcu_pose 飞机当前在世界坐标系下的位姿
     * @param cam_to_body_R 相机相对于机身的旋转
     * @param cam_to_body_T 相机相对于机身的平移
     * @return 世界坐标系下的位置
     */
    static Eigen::Vector3d cameraToWorld(const Eigen::Vector3d& p_cam, const geometry_msgs::Pose& fcu_pose, const Eigen::Matrix3d& cam_R_body, const Eigen::Vector3d& cam_T_body);

    static double getYawFromQuat(const geometry_msgs::Quaternion& q);

    /**
     * @brief 将机身坐标系下的点转换到世界坐标系
     */
    static Eigen::Vector3d bodyToWorld(const Eigen::Vector3d& body_point,
                                      const geometry_msgs::Pose& fcu_pose);
};

#endif // POSE_UTILS_H
