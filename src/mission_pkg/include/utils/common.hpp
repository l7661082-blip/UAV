/******************************************************************************
 * @file       drone_action_node.h
 * @brief      行为树节点基类 - 封装无人机控制的通用功能
 *
 * @details    为所有无人机行为树节点提供统一的状态访问、控制指令发布等
 *
 * @author     郭定<1604289397@qq.com>
 * @date       2026-01-06
 ******************************************************************************/
#ifndef COMMON_H
#define COMMON_H

#include <ros/ros.h>

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/condition_node.h>
#include <behaviortree_cpp/control_node.h>
#include <behaviortree_cpp/decorator_node.h>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/bt_cout_logger.h>


#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/State.h>

namespace BT
{

struct Position3D
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
  Position3D(const double &_x, const double &_y, const double &_z)
  {
    x = _x;
    y = _y;
    z = _z;
  }
  Position3D() {}
  Position3D &operator=(const Position3D &tmp)
  {
    x = tmp.x;
    y = tmp.y;
    z = tmp.z;
    return *this;
  }
  friend std::ostream &operator<<(std::ostream &out, Position3D pos);
};
inline std::ostream &operator<<(std::ostream &out, Position3D pos)
{
  out << pos.x << ";" << pos.y << ";" << pos.z;
  return out;
}



/**
 * @brief 像素目标（包含中心坐标和尺寸）
 */
struct PixelTarget
{
  double center_x{0.0};
  double center_y{0.0};
  double width{0.0};
  double height{0.0};
  double timestamp{0.0};  // ROS时间戳
  bool is_detected{false}; // 是否检测到有效目标
  
  PixelTarget() {}
  

      
  // 完整构造函数，默认 valid=false 以策安全
  PixelTarget(double cx, double cy, double w, double h, double ts, bool valid = false)
      : center_x(cx), center_y(cy), width(w), height(h), timestamp(ts), is_detected(valid) {}
  
  double area() const { return width * height; }
  
  friend std::ostream &operator<<(std::ostream &out, PixelTarget target);
};
inline std::ostream &operator<<(std::ostream &out, PixelTarget target)
{
  out << "center:(" << target.center_x << "," << target.center_y 
      << ") size:(" << target.width << "x" << target.height << ")"
      << " ts:" << target.timestamp
      << " valid:" << (target.is_detected ? "YES" : "NO");
  return out;
}

/**
 * 把参数字符串分割成PixelTarget结构
 */
template <> inline PixelTarget convertFromString(StringView str)
{
    auto parts = splitString(str, ';');
    if (parts.size() != 4)
    {
        throw RuntimeError("invalid input for PixelTarget (expected cx;cy;width;height)");
    }
    PixelTarget output;
    output.center_x = convertFromString<double>(parts[0]);
    output.center_y = convertFromString<double>(parts[1]);
    output.width = convertFromString<double>(parts[2]);
    output.height = convertFromString<double>(parts[3]);
    output.is_detected = true; // 字符串解析的通常是有效目标（手动指定）
    return output;
}

/**
 * 把参数字符穿分割成Position3D结构，在使用getInput解析参数时，BT内会自动调用这个内联函数
 */
template <> inline Position3D convertFromString(StringView str)
{
  auto parts = splitString(str, ';');
  if(parts.size() != 3)
  {
    throw RuntimeError("invalid input)");
  }
  else
  {
    Position3D output;
    output.x = convertFromString<double>(parts[0]);
    output.y = convertFromString<double>(parts[1]);
    output.z = convertFromString<double>(parts[2]);
    return output;
  }
}



}  // namespace BT
#endif  // COMMON_H
