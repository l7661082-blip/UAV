/******************************************************************************
 * @file       target_locator.cpp
 * @brief      TargetLocator 节点实现 - 完整 3D 定位
 *
 * @details    包含物理尺寸估算、遮挡检测、3D转换、位置验证、历史平滑
 *
 * @author     郭定<1604289397@qq.com>
 * @date       2026-01-13
 *****************************************************************************/
#include "nodes/perception/target_locator.h"

TargetLocator::TargetLocator(const std::string& name, const NodeConfig& config)
    : TargetDetectorBase(name, config) {
    
    auto nh = mct->getROSHandle();
    
    // 读取相机内参
    nh->param<double>("fx", fx, 320.0);
    nh->param<double>("fy", fy, 320.0);
    nh->param<double>("cx", cx, 320.0);
    nh->param<double>("cy", cy, 240.0);
    
    image_width = cx * 2.0;
    image_height = cy * 2.0;

    // 读取相机外参
    std::vector<double> cam2body_R_vec, cam2body_T_vec;
    
    if (!nh->getParam("cam2body_R", cam2body_R_vec) || cam2body_R_vec.size() != 9) {
        ROS_ERROR("Failed to get 'cam2body_R' parameter");
        throw std::runtime_error("Missing cam2body_R parameter");
    }
    cam_R_body << cam2body_R_vec[0], cam2body_R_vec[1], cam2body_R_vec[2],
                  cam2body_R_vec[3], cam2body_R_vec[4], cam2body_R_vec[5],
                  cam2body_R_vec[6], cam2body_R_vec[7], cam2body_R_vec[8];
    
    if (!nh->getParam("cam2body_T", cam2body_T_vec) || cam2body_T_vec.size() != 3) {
        ROS_ERROR("Failed to get 'cam2body_T' parameter");
        throw std::runtime_error("Missing cam2body_T parameter");
    }
    cam_T_body << cam2body_T_vec[0], cam2body_T_vec[1], cam2body_T_vec[2];
    
    best_estimate = Eigen::Vector3d(0, 0, 0);
}

PortsList TargetLocator::providedPorts() {
    auto ports = baseProvidedPorts();
    ports.insert(InputPort<double>("object_width", 1.3, "目标物体的物理宽度(米)"));
    ports.insert(InputPort<double>("object_height", 1.3, "目标物体的物理高度(米)"));
    ports.insert(InputPort<std::string>("occlusion_direction", "none", "遮挡方向: left/right/none"));
    ports.insert(InputPort<Position3D>("expected_pos", "预期目标位置(x,y,z)"));
    ports.insert(InputPort<double>("xy_tolerance", 1.0, "xy平面位置容差(米)"));
    ports.insert(InputPort<double>("z_tolerance", 1.0, "z轴位置容差(米)"));
    ports.insert(InputPort<int>("filter_window_size", 10, "平滑窗口大小"));
    ports.insert(OutputPort<Position3D>("position_3d", "3D世界坐标(x;y;z)"));
    return ports;
}

NodeStatus TargetLocator::onStart() {
    // 调用基类 onStart
    NodeStatus base_status = TargetDetectorBase::onStart();
    
    // 获取 3D 定位专用参数
    auto width_res = getInput<double>("object_width");
    physical_width = width_res.value();

    auto height_res = getInput<double>("object_height");
    physical_height = height_res.value();

    expected_aspect_ratio = physical_width / physical_height;

    auto dir_res = getInput<std::string>("occlusion_direction");
    occlusion_direction = dir_res.value();

    auto exp_res = getInput<Position3D>("expected_pos");
    if (exp_res) {
        expected_pos = exp_res.value();
        auto xy_tol_res = getInput<double>("xy_tolerance");
        xy_tolerance = xy_tol_res.value();
        auto z_tol_res = getInput<double>("z_tolerance");
        z_tolerance = z_tol_res.value();
    } else {
        xy_tolerance = 1000.0;
        z_tolerance = 1000.0;
    }

    auto window_res = getInput<int>("filter_window_size");
    filter_window_size = window_res.value();

    position_history.clear();
    
    ROS_INFO("[%s]: 3D Locator - size: %.2f x %.2f, ratio: %.2f, filter: %d", 
             name().c_str(), physical_width, physical_height, expected_aspect_ratio, filter_window_size);

    return base_status;
}

NodeStatus TargetLocator::handleDetectedObject(const common_msgs::Obj& obj) {
    // 1. FOV 边界检测
    if (isPartiallyOutOfView(obj)) {
        return NodeStatus::RUNNING;
    }
    
    // 2. 计算像素尺寸和实际长宽比
    double pixel_width = obj.right_bottom_x - obj.left_top_x;
    double pixel_height = obj.right_bottom_y - obj.left_top_y;
    
    if (pixel_width <= 0 || pixel_height <= 0) return NodeStatus::RUNNING;
    
    double actual_aspect_ratio = pixel_width / pixel_height;
    
    // 3. 遮挡检测
    const double occlusion_threshold = 0.75;
    bool is_occluded = (actual_aspect_ratio < expected_aspect_ratio * occlusion_threshold);
    
    // 4. 距离估算
    double distance;
    double compensated_center_x = obj.center_x;
    
    if (!is_occluded) {
        // 无遮挡：双尺寸估算
        double dist_from_width = (physical_width * fx) / pixel_width;
        double dist_from_height = (physical_height * fy) / pixel_height;
        
        double dist_diff = std::abs(dist_from_width - dist_from_height);
        if (dist_diff > 0.5) {
            return NodeStatus::RUNNING;
        }
        
        distance = (dist_from_width + dist_from_height) / 2.0;
        
    } else {
        // 有遮挡：只用高度估算 + 补偿中心
        distance = (physical_height * fy) / pixel_height;
        
        double full_pixel_width = pixel_height * expected_aspect_ratio;
        
        if (occlusion_direction == "right") {
            compensated_center_x = obj.left_top_x + full_pixel_width / 2.0;
        } else if (occlusion_direction == "left") {
            compensated_center_x = obj.right_bottom_x - full_pixel_width / 2.0;
        }
        
        ROS_INFO_THROTTLE(2.0, "[%s]: Occlusion detected! direction: %s, center: %.1f->%.1f",
                         name().c_str(), occlusion_direction.c_str(), 
                         static_cast<double>(obj.center_x), compensated_center_x);
    }
    
    // 5. 计算相机坐标
    Eigen::Vector3d p_cam;
    p_cam.x() = (compensated_center_x - cx) * distance / fx;
    p_cam.y() = (obj.center_y - cy) * distance / fy;
    p_cam.z() = distance;
    
    
    // 6. 转换到世界坐标 (ENU，与 fcu_odom 一致)
    Eigen::Vector3d p_world = PoseUtils::cameraToWorld(
        p_cam, fcu_odom_ptr->pose.pose, cam_R_body, cam_T_body);
    
    ROS_INFO_THROTTLE(1.0, "[%s]: cameraToWorld -> (%.2f, %.2f, %.2f)",
                     name().c_str(), p_world.x(), p_world.y(), p_world.z());
    
    // 7. 位置验证
    if (!isWithinExpectedRange(p_world)) {
        ROS_WARN_THROTTLE(2.0, "[%s]: Position out of range (%.2f, %.2f, %.2f)", 
                         name().c_str(), p_world.x(), p_world.y(), p_world.z());
        return NodeStatus::RUNNING;
    }
    
    // 8. 添加到历史
    addToHistory(p_world);
    
    // 9. 检查历史数据是否足够
    const int min_history_size = 10;
    if (position_history.size() < min_history_size) {
        ROS_INFO_THROTTLE(2.0, "[%s]: Collecting frames: %zu/%d", 
                         name().c_str(), position_history.size(), min_history_size);
        return NodeStatus::RUNNING;
    }
    
    // 10. 输出平滑后的 3D 坐标
    Eigen::Vector3d smoothed_pos = getSmoothedPosition();
    
    Position3D pos_3d;
    pos_3d.x = smoothed_pos.x();
    pos_3d.y = smoothed_pos.y();
    pos_3d.z = smoothed_pos.z();
    setOutput("position_3d", pos_3d);
    ROS_INFO("[%s]: Target located at (%.2f, %.2f, %.2f)", 
             name().c_str(), pos_3d.x, pos_3d.y, pos_3d.z);
    
    ROS_INFO("[%s]: Target located at (%.2f, %.2f, %.2f) - SUCCESS!", 
             name().c_str(), pos_3d.x, pos_3d.y, pos_3d.z);
    
    return NodeStatus::SUCCESS;  // 一次性定位，完成后退出
}

// 辅助函数实现
bool TargetLocator::isPartiallyOutOfView(const common_msgs::Obj& obj) {
    const double margin = 5.0;
    return obj.left_top_x < margin || 
           obj.right_bottom_x > (image_width - margin) ||
           obj.left_top_y < margin || 
           obj.right_bottom_y > (image_height - margin);
}

bool TargetLocator::isWithinExpectedRange(const Eigen::Vector3d& pos) {
    double dx = pos.x() - expected_pos.x;
    double dy = pos.y() - expected_pos.y;
    double dz = pos.z() - expected_pos.z;
    
    double xy_dist = std::sqrt(dx*dx + dy*dy);
    
    return xy_dist <= xy_tolerance && std::abs(dz) <= z_tolerance;
}

void TargetLocator::addToHistory(const Eigen::Vector3d& pos) {
    position_history.push_back(pos);
    
    while (static_cast<int>(position_history.size()) > filter_window_size) {
        position_history.pop_front();
    }
}

Eigen::Vector3d TargetLocator::getSmoothedPosition() {
    if (position_history.empty()) {
        return best_estimate;
    }
    
    Eigen::Vector3d sum(0, 0, 0);
    for (const auto& pos : position_history) {
        sum += pos;
    }
    
    best_estimate = sum / position_history.size();
    return best_estimate;
}

BT_REGISTER_NODES(factory) {
    factory.registerNodeType<TargetLocator>("TargetLocator");
}
