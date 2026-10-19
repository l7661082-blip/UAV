/******************************************************************************
 * @file       mavros_cnt.cpp
 * @brief      MAVROS 连接器实现
 * 
 * @details    实现与 MAVROS 的通信、状态管理、指令发布及安全边界保护
 * 
 * @author     郭定<1604289397@qq.com>
 * @date       2026-01-09
 *****************************************************************************/
#include "mavros_cnt.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>



std::shared_ptr<MavRosConnect>
MavRosConnect::getInstance(std::shared_ptr<ros::NodeHandle> nh)
{
  // 使用 std::call_once 确保线程安全
  std::call_once(initFlag, [&]() {
    mavros = std::shared_ptr<MavRosConnect>(new MavRosConnect(nh));
    mavros->InitMavRosConnect();
  });
  count_++;
  return mavros;  
}

void MavRosConnect::InitMavRosConnect()
{
  std::cout << "in initail ..." << std::endl;
  if(nh_ == nullptr)
  {
    ROS_ERROR("MavRosConnect node handle is empty");
    return;
  }
  nh_->param<std::string>("fcu_state_topic", param.fcu_state_topic,
                          "mavros/state");
  nh_->param<std::string>("pos_tgt_topic", param.pos_tgt_topic,
                          "mavros/setpoint_raw/local");
  nh_->param<std::string>("odom_topic", param.fcu_odom_topic,
                          "/rflysim/uav1/local/odom");
  nh_->param<std::string>("set_mode_service", param.set_mode_service,
                          "mavros/set_mode");
  nh_->param<std::string>("arm_service", param.arm_service,
                          "mavros/cmd/arming");

  fcu_state_sub = nh_->subscribe<mavros_msgs::State>(
    param.fcu_state_topic, 10,
    bind(&MavRosConnect::FcuStateCallback, mavros, _1));

  fcu_odom_sub = nh_->subscribe<nav_msgs::Odometry>(
    param.fcu_odom_topic, 10,
    bind(&MavRosConnect::FcuOdomCallback, mavros, _1));

  tgt_pose_pub =
    nh_->advertise<mavros_msgs::PositionTarget>(param.pos_tgt_topic, 10);

  att_pub =
    nh_->advertise<mavros_msgs::AttitudeTarget>(param.att_tgt_topic, 10);

  set_mode_client =
    nh_->serviceClient<mavros_msgs::SetMode>(param.set_mode_service);
  arming_client =
    nh_->serviceClient<mavros_msgs::CommandBool>(param.arm_service);

  // 加载安全边界参数
  nh_->param<bool>("safety/enabled", safety_boundary_.enabled, false);
  nh_->param<double>("safety/x_min", safety_boundary_.x_min, -10.0);
  nh_->param<double>("safety/x_max", safety_boundary_.x_max, 10.0);
  nh_->param<double>("safety/y_min", safety_boundary_.y_min, -10.0);
  nh_->param<double>("safety/y_max", safety_boundary_.y_max, 10.0);
  nh_->param<double>("safety/z_min", safety_boundary_.z_min, 0.0);
  nh_->param<double>("safety/z_max", safety_boundary_.z_max, 5.0);

  if (safety_boundary_.enabled) {
    ROS_INFO("Safety boundary enabled: X[%.1f, %.1f] Y[%.1f, %.1f] Z[%.1f, %.1f]",
             safety_boundary_.x_min, safety_boundary_.x_max,
             safety_boundary_.y_min, safety_boundary_.y_max,
             safety_boundary_.z_min, safety_boundary_.z_max);
  }

  // 初始化定时发布 (50Hz)
  has_init_cmd = false;
  last_safety_log_time_ = ros::Time(0);
  node_start_time_ = ros::Time::now();
  cmd_timer = nh_->createTimer(ros::Duration(0.02), &MavRosConnect::cmdPublishLoop, this);
}

const MavRosConnect::Param *MavRosConnect::getParam()
{
  return &param;
}

const nav_msgs::Odometry *MavRosConnect::getFcuOdom()
{
  return &fcu_odom;
}


const mavros_msgs::State *MavRosConnect::getFcuState()
{
  return &fcu_state;
}

ros::ServiceClient *MavRosConnect::getSetModeClient()
{
  return &set_mode_client;
}

ros::ServiceClient *MavRosConnect::getArmClient()
{
  return &arming_client;
}

ros::Publisher *MavRosConnect::getPosTgtPublisher()
{
  return &tgt_pose_pub;
}

ros::Publisher *MavRosConnect::getAttTgtPublisher()
{
  return &att_pub;
}



std::shared_ptr<ros::NodeHandle> MavRosConnect::getROSHandle()
{
  return nh_;
}

std::mutex &MavRosConnect::GetPoseMtx()
 {
    return pose_mtx;
 }
void MavRosConnect::FcuStateCallback(const mavros_msgs::State::ConstPtr &msg)
{
  std::lock_guard<std::mutex> lock(pose_mtx);
  if (!fcu_state_received_) {
    fcu_state_received_ = true;
    ROS_INFO("[MavRosConnect] Flight controller state connected (mavros/state)");
  }
  fcu_state = *msg;
}

void MavRosConnect::FcuOdomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
  if (!fcu_odom_received_) {
    fcu_odom_received_ = true;
    ROS_INFO("[MavRosConnect] Drone localization connected (odom)");
  }
  fcu_odom = *msg;
}

void MavRosConnect::updateCommand(const mavros_msgs::PositionTarget &cmd)
{
    std::lock_guard<std::mutex> lock(cmd_mtx);
    current_cmd = cmd;
    has_init_cmd = true;
}

void MavRosConnect::cmdPublishLoop(const ros::TimerEvent &event)
{
    // 跟踪启动后的延迟，避免刚启动时立即警告
    constexpr double STARTUP_DELAY_SEC = 2.0;
    bool is_startup_period = (ros::Time::now() - node_start_time_).toSec() < STARTUP_DELAY_SEC;

    // 检查是否接收到飞控状态信息
    if (!fcu_state_received_ || !fcu_odom_received_) {
        if (!is_startup_period) {
            if (!fcu_state_received_) ROS_WARN_THROTTLE(2.0, "[MavRosConnect] Waiting for FCU state...");
            if (!fcu_odom_received_) ROS_WARN_THROTTLE(2.0, "[MavRosConnect] Waiting for FCU odom...");
        }
        return;
    }

    // 提前检查是否已初始化指令 (用户需求)
    if (!has_init_cmd) {
        return;
    }

    // 安全边界检查
    if (safety_boundary_.enabled && !isWithinBoundary()) {
        ros::Time now = ros::Time::now();
        
        // 1. 首次触发越界时，锁定当前位置作为悬停点
        if (!hover_locked_) {
            hover_pos_ = fcu_odom.pose.pose.position;
            tf2::Quaternion q;
            tf2::fromMsg(fcu_odom.pose.pose.orientation, q);
            double r, p, y;
            tf2::Matrix3x3(q).getRPY(r, p, y);
            hover_yaw_ = y;
            hover_locked_ = true;
            
            ROS_ERROR("[Safety] DRONE OUT OF BOUNDARY! Triggering Hover Lock at (%.2f, %.2f, %.2f)",
                     hover_pos_.x, hover_pos_.y, hover_pos_.z);
        } else {
            // 持续报错 (2秒一次)
            if ((now - last_safety_log_time_).toSec() > 2.0) {
                ROS_ERROR("[Safety] OUT OF BOUNDARY! Hovering at locked pos (%.2f, %.2f, %.2f)",
                         hover_pos_.x, hover_pos_.y, hover_pos_.z);
                last_safety_log_time_ = now;
            }
        }
        
        // 构造悬停指令并更新到 current_cmd
        mavros_msgs::PositionTarget hover_cmd;
        hover_cmd.header.stamp = now;
        hover_cmd.header.frame_id = "safety_hover";
        hover_cmd.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        hover_cmd.type_mask = mavros_msgs::PositionTarget::IGNORE_VX |
                              mavros_msgs::PositionTarget::IGNORE_VY |
                              mavros_msgs::PositionTarget::IGNORE_VZ |
                              mavros_msgs::PositionTarget::IGNORE_YAW_RATE |
                              mavros_msgs::PositionTarget::IGNORE_AFX |
                              mavros_msgs::PositionTarget::IGNORE_AFY |
                              mavros_msgs::PositionTarget::IGNORE_AFZ |
                              mavros_msgs::PositionTarget::FORCE;
        // ENU → NED 转换（与 publishPosition 保持一致）
        hover_cmd.position.x = -hover_pos_.y;   // NED x = -ENU y
        hover_cmd.position.y = hover_pos_.x;     // NED y = ENU x
        hover_cmd.position.z = hover_pos_.z;
        hover_cmd.yaw = hover_yaw_ + M_PI / 2.0;
        
        updateCommand(hover_cmd);
    } else {
        // 如果回到安全区域，重置锁定标志
        if (hover_locked_) {
            ROS_INFO("[Safety] Drone returned to safe area. Hover lock released.");
            hover_locked_ = false;
        }
    }

    // 统一发布 current_cmd
    std::lock_guard<std::mutex> lock(cmd_mtx);
    current_cmd.header.stamp = ros::Time::now();
    tgt_pose_pub.publish(current_cmd);
}

bool MavRosConnect::isWithinBoundary() const
{
    // odom 和安全参数都是 ENU，直接比较
    double x = fcu_odom.pose.pose.position.x;
    double y = fcu_odom.pose.pose.position.y;
    double z = fcu_odom.pose.pose.position.z;
    return (x >= safety_boundary_.x_min && x <= safety_boundary_.x_max &&
            y >= safety_boundary_.y_min && y <= safety_boundary_.y_max &&
            z >= safety_boundary_.z_min && z <= safety_boundary_.z_max);
}

MavRosConnect::MavRosConnect(std::shared_ptr<ros::NodeHandle> nh)
{
  nh_ = nh;
}
