/******************************************************************************
 * @file       aruco_detect_node.cpp
 * @brief      ArUco 二维码识别独立 ROS 节点（带开关控制）
 *
 * @details    平时空转不处理图像，行为树通过 Service 开启识别后才处理
 *
 *             服务接口:
 *               /aruco/enable  (std_srvs/SetBool)  开启/关闭检测
 *
 *             发布话题:
 *               /aruco/detected_id  (std_msgs/Int32)  检测到的 ArUco ID
 *               /aruco/debug_image  (sensor_msgs/Image) 调试图像（可选）
 *
 *             订阅话题:
 *               ~image_topic (default: /rflysim/sensor2/img_rgb)  RGB 图像
 *
 * @date       2026-03-16
 ******************************************************************************/
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Int32.h>
#include <std_srvs/SetBool.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/aruco.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <mutex>
#include <atomic>

class ArucoDetectNode {
public:
    ArucoDetectNode(ros::NodeHandle& nh, ros::NodeHandle& nh_private) 
        : nh_(nh), has_detection_(false) {
        
        // 参数
        nh_private.param<std::string>("image_topic", image_topic_, "/rflysim/sensor2/img_rgb");
        nh_private.param<int>("dictionary_id", dictionary_id_, 10);  // DICT_6X6_250
        nh_private.param<bool>("show_debug", show_debug_, false);
        nh_private.param<std::string>("save_dir", save_dir_, "");
        
        // 默认是否开启检测（正式比赛默认 false，通过 /aruco/enable 服务触发）
        bool enabled_default;
        nh_private.param<bool>("enabled_default", enabled_default, false);
        enabled_ = enabled_default;
        
        // 初始化 ArUco 检测器
        aruco_dict_ = cv::aruco::getPredefinedDictionary(dictionary_id_);
        detector_params_ = cv::aruco::DetectorParameters::create();
        
        // 可调检测参数（通过 launch 文件配置）
        int thresh_win_min, thresh_win_max, thresh_win_step;
        double thresh_constant;
        nh_private.param<int>("adaptive_thresh_win_min", thresh_win_min, 3);
        nh_private.param<int>("adaptive_thresh_win_max", thresh_win_max, 53);
        nh_private.param<int>("adaptive_thresh_win_step", thresh_win_step, 4);
        nh_private.param<double>("adaptive_thresh_constant", thresh_constant, 7.0);
        detector_params_->adaptiveThreshWinSizeMin = thresh_win_min;
        detector_params_->adaptiveThreshWinSizeMax = thresh_win_max;
        detector_params_->adaptiveThreshWinSizeStep = thresh_win_step;
        detector_params_->adaptiveThreshConstant = thresh_constant;
        
        double min_perimeter, max_perimeter;
        nh_private.param<double>("min_marker_perimeter_rate", min_perimeter, 0.01);
        nh_private.param<double>("max_marker_perimeter_rate", max_perimeter, 4.0);
        detector_params_->minMarkerPerimeterRate = min_perimeter;
        detector_params_->maxMarkerPerimeterRate = max_perimeter;
        
        double poly_accuracy, corner_dist;
        nh_private.param<double>("polygonal_approx_accuracy", poly_accuracy, 0.05);
        nh_private.param<double>("min_corner_distance_rate", corner_dist, 0.02);
        detector_params_->polygonalApproxAccuracyRate = poly_accuracy;
        detector_params_->minCornerDistanceRate = corner_dist;
        
        int persp_pixel_per_cell;
        double persp_ignored_margin;
        nh_private.param<int>("perspective_remove_pixel_per_cell", persp_pixel_per_cell, 8);
        nh_private.param<double>("perspective_remove_ignored_margin", persp_ignored_margin, 0.2);
        detector_params_->perspectiveRemovePixelPerCell = persp_pixel_per_cell;
        detector_params_->perspectiveRemoveIgnoredMarginPerCell = persp_ignored_margin;
        
        // 角点亚像素精化（关键！默认不精化，导致透视校正不准，bit读取错误）
        detector_params_->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
        detector_params_->cornerRefinementWinSize = 5;
        detector_params_->cornerRefinementMaxIterations = 30;
        detector_params_->cornerRefinementMinAccuracy = 0.1;
        
        // 纠错率（0.6=默认, 适当提高到0.8容忍仿真渲染导致的bit误差）
        double error_correction;
        nh_private.param<double>("error_correction_rate", error_correction, 0.8);
        detector_params_->errorCorrectionRate = error_correction;
        
        // 发布器
        id_pub_ = nh_.advertise<std_msgs::Int32>("/aruco/detected_id", 10);
        debug_img_pub_ = nh_.advertise<sensor_msgs::Image>("/aruco/debug_image", 1);
        
        // 服务：开启/关闭检测
        enable_srv_ = nh_.advertiseService("/aruco/enable", 
                                            &ArucoDetectNode::enableCallback, this);
        
        // 仅在 enabled_default=true 时才订阅图像（否则完全不订阅，0 CPU）
        if (enabled_) {
            subscribeImage();
        }
        
        ROS_INFO("========================================");
        ROS_INFO("[ArucoDetectNode] Node started");
        ROS_INFO("[ArucoDetectNode] Image topic: %s", image_topic_.c_str());
        ROS_INFO("[ArucoDetectNode] Dictionary ID: %d (DICT_6X6_250=10)", dictionary_id_);
        ROS_INFO("[ArucoDetectNode] Detection: %s", enabled_ ? "ENABLED" : "DISABLED (call /aruco/enable to start)");
        ROS_INFO("========================================");
    }

private:
    // ==================== 图像订阅管理 ====================
    void subscribeImage() {
        if (!img_sub_) {
            img_sub_ = nh_.subscribe(image_topic_, 1, &ArucoDetectNode::imageCallback, this);
            ROS_INFO("[ArucoDetectNode] Subscribed to image topic: %s", image_topic_.c_str());
        }
    }
    
    void unsubscribeImage() {
        if (img_sub_) {
            img_sub_.shutdown();
            ROS_INFO("[ArucoDetectNode] Unsubscribed from image topic (saving CPU)");
        }
    }
    
    // ==================== 服务回调 ====================
    bool enableCallback(std_srvs::SetBool::Request& req, 
                        std_srvs::SetBool::Response& res) {
        enabled_ = req.data;
        has_detection_ = false;  // 重置检测状态
        
        if (enabled_) {
            subscribeImage();
            ROS_INFO("[ArucoDetectNode] >>> Detection ENABLED <<<");
            res.message = "ArUco detection enabled";
        } else {
            unsubscribeImage();
            ROS_INFO("[ArucoDetectNode] >>> Detection DISABLED <<<");
            res.message = "ArUco detection disabled";
        }
        res.success = true;
        return true;
    }
    
    // ==================== 图像回调 ====================
    void imageCallback(const sensor_msgs::Image::ConstPtr& msg) {
        // 未开启检测时，直接返回（不消耗 CPU）
        if (!enabled_) {
            return;
        }
        
        // 转换为 OpenCV 格式
        cv::Mat image;
        try {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
            image = cv_ptr->image;
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR_THROTTLE(1.0, "[ArucoDetectNode] cv_bridge exception: %s", e.what());
            return;
        }
        
        if (image.empty()) return;
        
        // 灰度转换
        cv::Mat gray;
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
        
        // 直接使用原始灰度图检测（不做预处理，减少 CPU 消耗）
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;
        std::vector<std::vector<cv::Point2f>> rejected;
        
        cv::aruco::detectMarkers(gray, aruco_dict_, corners, ids, detector_params_, rejected);
        
        if (!ids.empty()) {
            ROS_INFO_THROTTLE(5.0, "[ArucoDetectNode] Detected (raw gray)");
        }
        
        // 仅在需要时生成调试图像（避免不必要的 clone + 绘制）
        bool need_debug = show_debug_ || (debug_img_pub_.getNumSubscribers() > 0);
        cv::Mat debug_img;
        if (need_debug || !ids.empty()) {
            debug_img = image.clone();
            if (!ids.empty()) {
                cv::aruco::drawDetectedMarkers(debug_img, corners, ids);
            }
            if (need_debug) {
                // 绘制被拒绝的候选区域（红色框）
                for (auto& rej : rejected) {
                    std::vector<cv::Point> pts;
                    for (auto& p : rej) pts.push_back(cv::Point(p.x, p.y));
                    cv::polylines(debug_img, pts, true, cv::Scalar(0, 0, 255), 1);
                }
                std::string status = ids.empty() ? 
                    "NO MATCH (rejected: " + std::to_string(rejected.size()) + ")" :
                    "ID: " + std::to_string(ids[0]);
                cv::putText(debug_img, status, cv::Point(10, 30), 
                            cv::FONT_HERSHEY_SIMPLEX, 0.8, 
                            ids.empty() ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 255, 0), 2);
                
                // 发布调试图像
                if (debug_img_pub_.getNumSubscribers() > 0) {
                    auto debug_msg = cv_bridge::CvImage(msg->header, "bgr8", debug_img).toImageMsg();
                    debug_img_pub_.publish(debug_msg);
                }
                // 可选：显示窗口
                if (show_debug_) {
                    cv::imshow("ArUco Detection", debug_img);
                    cv::waitKey(1);
                }
            }
        }
        
        // 检测到标记 → 发布结果 → 自动关闭检测
        if (!ids.empty()) {
            int detected_id = ids[0];
            
            // 发布 ID
            std_msgs::Int32 id_msg;
            id_msg.data = detected_id;
            id_pub_.publish(id_msg);
            
            // 醒目终端输出（ANSI 颜色）：亮黄色粗体 + 绿色高亮框
            // \033[1;33m = 亮黄色粗体  \033[1;32m = 亮绿色粗体  \033[0m = 重置
            ROS_INFO("\033[1;33m╔══════════════════════════════════════════╗\033[0m");
            ROS_INFO("\033[1;33m║  \033[1;32m*** ArUco Marker Detected! ***\033[1;33m        ║\033[0m");
            ROS_INFO("\033[1;33m║                                          ║\033[0m");
            ROS_INFO("\033[1;33m║    \033[1;37mDetected ID: \033[1;36m%-4d\033[1;33m                    ║\033[0m", detected_id);
            ROS_INFO("\033[1;33m╚══════════════════════════════════════════╝\033[0m");
            
            // 保存图片
            if (!save_dir_.empty() && !debug_img.empty()) {
                std::string save_path = save_dir_ + "/aruco_id_" + 
                                        std::to_string(detected_id) + ".jpg";
                cv::imwrite(save_path, debug_img);
                ROS_INFO("[ArucoDetectNode] Image saved: %s", save_path.c_str());
            }
            
            has_detection_ = true;
            
            // ★ 检测成功后自动关闭：取消订阅图像，释放 CPU
            enabled_ = false;
            unsubscribeImage();
            ROS_INFO("[ArucoDetectNode] Detection auto-disabled after success (saving CPU)");
        } else {
            ROS_INFO_THROTTLE(2.0, "[ArucoDetectNode] Searching... no marker found (rejected: %zu)", rejected.size());
        }
    }
    
    // ==================== 成员变量 ====================
    ros::NodeHandle nh_;
    ros::Subscriber img_sub_;
    ros::Publisher id_pub_;
    ros::Publisher debug_img_pub_;
    ros::ServiceServer enable_srv_;
    
    // ArUco
    cv::Ptr<cv::aruco::Dictionary> aruco_dict_;
    cv::Ptr<cv::aruco::DetectorParameters> detector_params_;
    int dictionary_id_;
    
    // 状态
    std::atomic<bool> enabled_;
    bool has_detection_;
    bool show_debug_;
    std::string save_dir_;
    std::string image_topic_;  // 保存话题名，便于动态重新订阅
};


int main(int argc, char** argv) {
    ros::init(argc, argv, "aruco_detect_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    
    ArucoDetectNode node(nh, nh_private);
    
    ros::spin();
    return 0;
}
