#ifndef MISSION_PKG_FLIGHT_BOUNDARY_H
#define MISSION_PKG_FLIGHT_BOUNDARY_H

#include <algorithm>
#include <cmath>
#include <string>
#include <geometry_msgs/Point.h>
#include <ros/ros.h>

struct FlightBoundary {
    double x_min, x_max, y_min, y_max;

    static bool load(FlightBoundary& result) {
        ros::NodeHandle nh;
        const std::string base = "/referee_policy/boundary/flight/";
        bool found = nh.getParam(base + "x_min", result.x_min)
                  && nh.getParam(base + "x_max", result.x_max)
                  && nh.getParam(base + "y_min", result.y_min)
                  && nh.getParam(base + "y_max", result.y_max);
        if (!found || !std::isfinite(result.x_min) || !std::isfinite(result.x_max)
                   || !std::isfinite(result.y_min) || !std::isfinite(result.y_max)
                   || result.x_min >= result.x_max || result.y_min >= result.y_max) {
            ROS_ERROR("Missing or invalid /referee_policy/boundary/flight; refusing navigation goal");
            return false;
        }
        return true;
    }

    bool clamp(geometry_msgs::Point& point, const std::string& node) const {
        if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
            ROS_ERROR("[%s] Non-finite navigation goal rejected", node.c_str());
            return false;
        }
        const double old_x = point.x, old_y = point.y;
        point.x = std::clamp(point.x, x_min, x_max);
        point.y = std::clamp(point.y, y_min, y_max);
        if (point.x != old_x || point.y != old_y) {
            ROS_WARN("[%s] Navigation goal (%.3f, %.3f) clamped to (%.3f, %.3f)",
                     node.c_str(), old_x, old_y, point.x, point.y);
        }
        return true;
    }
};

#endif
