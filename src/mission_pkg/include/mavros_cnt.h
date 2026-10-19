/******************************************************************************
 * @file       mavros_cnt.h
 * @brief      MAVROS 连接管理器 - 统一管理无人机状态、定位和控制接口
 *
 * @details    该类采用单例模式，封装了与 MAVROS 的所有交互，包括订阅状态、
 *             发布控制指令以及安全边界检查。
 *
 * @author     郭定<1604289397@qq.com>
 * @date       2026-01-09
 *****************************************************************************/
#ifndef MAVROS_CNT_H
#define MAVROS_CNT_H
#include <ros/ros.h>
#include <utils/common.hpp>
#include <mutex>
#include <memory>

class MavRosConnect
{

  typedef struct param_
  {
    //下面的变量都是只读
    std::string fcu_state_topic;
    std::string pos_tgt_topic;
    std::string set_mode_service;
    std::string arm_service;

    std::string fcu_odom_topic;
    std::string att_tgt_topic;

    param_() {}
  } Param;

  // 安全边界参数
  struct SafetyBoundary {
    bool enabled = false;
    double x_min = -10.0;
    double x_max = 10.0;
    double y_min = -10.0;
    double y_max = 10.0;
    double z_min = 0.0;
    double z_max = 5.0;
  };

public:
  static std::shared_ptr<MavRosConnect>
               getInstance(std::shared_ptr<ros::NodeHandle> nh = nullptr);
  void         InitMavRosConnect();
  const Param *getParam();

  const nav_msgs::Odometry *getFcuOdom();
  const mavros_msgs::State *        getFcuState();
  std::mutex &GetPoseMtx();
  ros::ServiceClient *              getSetModeClient();
  ros::ServiceClient *              getArmClient();
  ros::Publisher *                  getPosTgtPublisher();
  ros::Publisher *                  getAttTgtPublisher();
  std::shared_ptr<ros::NodeHandle>  getROSHandle();

  // 更新控制目标 (由行为树节点调用)
  void updateCommand(const mavros_msgs::PositionTarget &cmd);



  //订阅飞控状态的回调函数
  void FcuStateCallback(const mavros_msgs::State::ConstPtr &msg);

  // 订阅飞控里程计数据,里面包含姿态，速度，加速度信息。可以方便提供给行为树应用
  void FcuOdomCallback(const nav_msgs::Odometry::ConstPtr &msg);

  // 定时发布控制循环
  void cmdPublishLoop(const ros::TimerEvent &event);

  // 安全边界检查
  bool isWithinBoundary() const;

  static int count_;
  static std::mutex pose_mtx;
private:
  MavRosConnect(std::shared_ptr<ros::NodeHandle> nh);
  MavRosConnect(const MavRosConnect &) = delete;
  MavRosConnect &operator=(const MavRosConnect &) = delete;

  static std::once_flag initFlag;

  static std::shared_ptr<MavRosConnect> mavros;
  //  static std::mutex                     mtx;
  
  Param                      param;
  mavros_msgs::State         fcu_state;
  nav_msgs::Odometry         fcu_odom;

  std::shared_ptr<ros::NodeHandle> nh_;
  ros::Subscriber                  fcu_odom_sub;
  ros::Subscriber                  fcu_state_sub;
  ros::Publisher                   tgt_pose_pub;
  ros::Publisher                   att_pub;
  ros::ServiceClient               set_mode_client;
  ros::ServiceClient               arming_client;

  // 控制指令缓存与同步
  mavros_msgs::PositionTarget current_cmd;
  std::mutex                  cmd_mtx;
  ros::Timer                  cmd_timer;
  bool                        has_init_cmd;

  // 安全边界
  SafetyBoundary safety_boundary_;
  
  // 悬停状态 (用于边界保护)
  bool hover_locked_ = false;  // 重命名以区别于 DroneActionNode
  geometry_msgs::Point hover_pos_;
  double hover_yaw_{0.0};
  
  ros::Time last_safety_log_time_;

  // 连接状态跟踪
  bool fcu_state_received_ = false;
  bool fcu_odom_received_ = false;
  ros::Time node_start_time_;
};

std::shared_ptr<MavRosConnect> MavRosConnect::mavros = nullptr;
std::once_flag                 MavRosConnect::initFlag;
int                            MavRosConnect::count_ = 0;
std::mutex MavRosConnect::pose_mtx;
#endif  // MAVROS_H
