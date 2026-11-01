/******************************************************************************
 * @file       bt_ros.cpp
 * @brief      行为树 ROS 主节点 - 加载和执行任务行为树
 * 
 * @details    支持两种运行模式：
 *             - static: 从 XML 文件加载并执行（原有行为）
 *             - llm:    监听 ROS Topic，接收 LLM 指令动态构建行为树执行
 * 
 * @author     郭定<1604289397@qq.com>
 * @date       2026-01-06
 *****************************************************************************/
#include <ros/ros.h>
#include <behaviortree_cpp/xml_parsing.h>
#include <boost/stacktrace.hpp>
#include <std_msgs/String.h>

#include <csignal>
#include <filesystem>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include <mutex>
#include <atomic>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>

#include "nlohmann_json.hpp"
#include "mavros_cnt.h"
#include "nodes/action/takeoff.h"
#include "nodes/action/land.h"

namespace fs = std::filesystem;
using namespace BT;
using json = nlohmann::json;

//==============================================================================
// 常量定义
//==============================================================================
namespace {
    constexpr int ASYNC_SPINNER_THREADS = 2;
    const std::string LIB_PREFIX = "lib";
    const std::string SO_EXTENSION = ".so";
}

//==============================================================================
// 信号处理
//==============================================================================

/**
 * @brief 信号处理函数 - 捕获崩溃信号并打印堆栈跟踪
 */
void signalHandler(int signum) {
    std::cerr << "\n[FATAL] Received signal " << signum << std::endl;
    std::cerr << "Stack trace:\n" << boost::stacktrace::stacktrace();
    std::exit(signum);
}

/**
 * @brief 注册信号处理器
 */
void registerSignalHandlers() {
    std::signal(SIGSEGV, signalHandler);
    std::signal(SIGABRT, signalHandler);
}

//==============================================================================
// 插件加载
//==============================================================================

/**
 * @brief 从 devel/lib 目录扫描并加载行为树插件 (.so 文件)
 * @return 插件文件的完整路径列表
 */
std::vector<std::string> loadPluginLibraries() {
    std::vector<std::string> lib_files;
    
    const char* cmake_prefix_path = std::getenv("CMAKE_PREFIX_PATH");
    if (!cmake_prefix_path) {
        ROS_ERROR("CMAKE_PREFIX_PATH environment variable not set!");
        return lib_files;
    }

    std::string prefix_path_str(cmake_prefix_path);
    size_t devel_pos = prefix_path_str.find("/devel");
    
    if (devel_pos == std::string::npos) {
        ROS_ERROR("Failed to find 'devel' in CMAKE_PREFIX_PATH: %s", prefix_path_str.c_str());
        return lib_files;
    }

    std::string devel_lib_path = prefix_path_str.substr(0, devel_pos + 6) + "/lib";
    
    try {
        for (const auto& entry : fs::directory_iterator(devel_lib_path)) {
            if (!entry.is_regular_file() || entry.path().extension() != SO_EXTENSION) {
                continue;
            }
            
            std::string filename = entry.path().filename().string();
            // 只加载非标准库文件（不以 "lib" 开头的 .so 文件）
            if (filename.substr(0, LIB_PREFIX.size()) != LIB_PREFIX) {
                lib_files.push_back(entry.path().string());
                ROS_DEBUG("Found plugin: %s", filename.c_str());
            }
        }
        ROS_INFO("Loaded %zu plugin(s) from: %s", lib_files.size(), devel_lib_path.c_str());
    } catch (const fs::filesystem_error& e) {
        ROS_ERROR("Filesystem error while loading plugins: %s", e.what());
    }

    if (lib_files.empty()) {
        ROS_WARN("No behavior tree plugins found in %s", devel_lib_path.c_str());
    }

    return lib_files;
}

/**
 * @brief 注册所有插件到行为树工厂
 */
void registerPlugins(BehaviorTreeFactory& factory, const std::vector<std::string>& plugin_files) {
    for (const auto& plugin_path : plugin_files) {
        try {
            factory.registerFromPlugin(plugin_path);
            ROS_DEBUG("Registered plugin: %s", plugin_path.c_str());
        } catch (const std::exception& e) {
            ROS_ERROR("Failed to register plugin %s: %s", plugin_path.c_str(), e.what());
        }
    }
}

//==============================================================================
// 配置管理
//==============================================================================

/**
 * @brief 行为树配置参数
 */
struct BehaviorTreeConfig {
    std::string tree_config_path;
    bool is_debug;
    bool generate_xml;
    std::string xml_output_file;
    std::string mode;  // "static" or "llm"
};

/**
 * @brief 从 ROS 参数服务器加载配置
 */
BehaviorTreeConfig loadConfiguration(ros::NodeHandle& nh_private) {
    BehaviorTreeConfig config;
    
    nh_private.param<std::string>("config_path", config.tree_config_path,
                                   "/home/zy/ws/src/mission_pkg/config/hitballon.xml");
    nh_private.param<bool>("is_debug", config.is_debug, false);
    nh_private.param<bool>("is_gen_xml", config.generate_xml, false);
    nh_private.param<std::string>("xml_file", config.xml_output_file, "cfg.xml");
    nh_private.param<std::string>("mode", config.mode, "static");
    
    ROS_INFO("Behavior Tree Config Path: %s", config.tree_config_path.c_str());
    ROS_INFO("Mode: %s", config.mode.c_str());
    
    return config;
}

//==============================================================================
// XML 生成
//==============================================================================

/**
 * @brief 生成行为树节点模型 XML 文件（用于调试和文档）
 */
bool generateNodeModelXML(const BehaviorTreeFactory& factory, const std::string& output_file) {
    try {
        auto xml_model = writeTreeNodesModelXML(factory, false);
        
        std::ofstream file(output_file);
        if (!file.is_open()) {
            ROS_ERROR("Failed to open file for writing: %s", output_file.c_str());
            return false;
        }
        
        file << xml_model;
        file.close();
        
        ROS_INFO("Successfully generated XML model: %s", output_file.c_str());
        return true;
        
    } catch (const std::exception& e) {
        ROS_ERROR("Failed to generate XML model: %s", e.what());
        return false;
    }
}

//==============================================================================
// LLM 指令 → 行为树 XML 动态构建
//==============================================================================

/**
 * @brief 将单条 LLM 指令转换为行为树 XML 片段
 * 
 * 支持的指令格式：
 *   takeoff
 *   cross_frame
 *   detect_aruco
 *   return_cross_frame
 *   hit_balloon:red / hit_balloon:purple / hit_balloon:blue
 *   land
 */
std::string commandToXmlNode(const std::string& command) {
    // 去除前后空白
    std::string cmd = command;
    cmd.erase(0, cmd.find_first_not_of(" \t\r\n"));
    cmd.erase(cmd.find_last_not_of(" \t\r\n") + 1);
    
    if (cmd.empty()) return "";

    // 读取参数的辅助 lambda
    ros::NodeHandle nh;
    auto getStr = [&](const std::string& key, const std::string& def) -> std::string {
        std::string val;
        if (!nh.getParam(key, val)) val = def;
        return val;
    };
    auto getDbl = [&](const std::string& key, double def) -> double {
        double val;
        if (!nh.getParam(key, val)) val = def;
        return val;
    };
    auto getInt = [&](const std::string& key, int def) -> int {
        int val;
        if (!nh.getParam(key, val)) val = def;
        return val;
    };

    // ---- takeoff ----
    if (cmd == "takeoff") {
        std::string goal = getStr("takeoff/goal", "0.0;0.0;1.0");
        double yaw_val = 0;
        bool has_yaw = nh.getParam("takeoff/yaw", yaw_val);
        if (has_yaw) {
            return "      <Takeoff goal=\"" + goal + "\" yaw=\"" + std::to_string(yaw_val) + "\"/>\n";
        }
        return "      <Takeoff goal=\"" + goal + "\"/>\n";
    }
    
    // ---- cross_frame: 识别方框 + 穿越 ----
    if (cmd == "cross_frame") {
        std::string observe   = getStr("cross_frame/observe_pos",   "0.0;-1.0;1.0");
        std::string delay     = std::to_string((int)getDbl("cross_frame/observe_delay_ms", 2000));
        std::string obj_w     = std::to_string(getDbl("cross_frame/object_width",  1.3));
        std::string obj_h     = std::to_string(getDbl("cross_frame/object_height", 1.3));
        std::string occ_dir   = getStr("cross_frame/occlusion_direction", "right");
        std::string exp_pos   = getStr("cross_frame/expected_pos",  "3.25;-0.4;1.0");
        std::string z_tol     = std::to_string(getDbl("cross_frame/z_tolerance", 0.5));
        std::string filter_w  = std::to_string((int)getDbl("cross_frame/filter_window_size", 20));
        std::string offset_z  = std::to_string(getDbl("cross_frame/offset_z", 1.0));
        
        return 
            "      <!-- 移动到观察位置 -->\n"
            "      <GoTo goal=\"" + observe + "\"/>\n"
            "      <Delay delay_msec=\"" + delay + "\">\n"
            "        <AlwaysSuccess/>\n"
            "      </Delay>\n"
            "      <!-- 识别方框位置 -->\n"
            "      <TargetLocator object_name=\"square_frame\" object_width=\"" + obj_w + "\" object_height=\"" + obj_h + "\" "
            "occlusion_direction=\"" + occ_dir + "\" expected_pos=\"" + exp_pos + "\" "
            "xy_tolerance=\"1.0\" z_tolerance=\"" + z_tol + "\" "
            "confidence_threshold=\"0.7\" "
            "position_3d=\"{frame_center}\" filter_window_size=\"" + filter_w + "\"/>\n"
            "      <!-- 规划穿过方框 -->\n"
            "      <PlanningControl goal=\"{frame_center}\" offset_z=\"" + offset_z + "\" final_position=\"{planning_end_pos}\"/>\n";
    }
    
    // ---- detect_aruco: 飞到二维码上方识别 ----
    if (cmd == "detect_aruco") {
        std::string hover = getStr("aruco/hover_pos", "4.5;0.5;1.2");
        return
            "      <GoTo goal=\"" + hover + "\"/>\n"
            "      <Parallel success_count=\"2\" failure_count=\"1\">\n"
            "        <GoTo goal=\"" + hover + "\"/>\n"
            "        <ArucoDetector aruco_id=\"{detected_aruco_id}\"/>\n"
            "      </Parallel>\n";
    }
    
    // ---- return_cross_frame: 返回并反向穿越方框 ----
    if (cmd == "return_cross_frame") {
        std::string turn_pos     = getStr("return_cross_frame/turn_pos",   "4.5;0.5;1.8");
        double      turn_yaw     = getDbl("return_cross_frame/turn_yaw",   3.14159);
        std::string body_fwd     = getStr("return_cross_frame/body_forward", "0.3;0.0;0.0");
        double      exit_yaw     = getDbl("return_cross_frame/exit_yaw",   0.0);
        std::string offset_z    = std::to_string(getDbl("cross_frame/offset_z", 1.0));
        
        return
            "      <!-- 旋转180度 -->\n"
            "      <GoTo goal=\"" + turn_pos + "\" yaw=\"" + std::to_string(turn_yaw) + "\"/>\n"
            "      <!-- 返回规划终点 -->\n"
            "      <GoTo goal=\"{planning_end_pos}\"/>\n"
            "      <!-- 反向穿越方框 -->\n"
            "      <PlanningControl goal=\"{frame_center}\" offset_z=\"" + offset_z + "\"/>\n"
            "      <!-- 离框平面至少 0.5m 后，仅前进 0.3m 并转向 -->\n"
            "      <GoTo goal=\"" + body_fwd + "\" frame=\"1\" clearance_from=\"{frame_center}\" "
            "min_frame_clearance=\"0.5\" yaw=\"" + std::to_string(exit_yaw) + "\"/>\n";
    }
    
    // ---- hit_balloon:颜色 ----
    if (cmd == "hit_balloon:red" || cmd == "hit_balloon:purple" ||
        cmd == "hit_balloon:blue") {
        std::string color = cmd.substr(std::string("hit_balloon:").size());
        
        std::string zone;
        if (!nh.getParam("balloon_color_" + color, zone)) {
            ROS_WARN("Parameter 'balloon_color_%s' not set, using zone A as default.", color.c_str());
            zone = "A";
        }
        
        std::string param_prefix = "referee_policy/balloon_zone_" + zone;
        std::string approach_pos = getStr(param_prefix + "/approach_pos", "");
        std::string goto_pos = getStr(param_prefix + "/goal", "");
        double yaw_val = getDbl(param_prefix + "/yaw", 0.0);
        std::string yaw = std::to_string(yaw_val);
        std::string balloon_name = color + "_balloon";
        
        // VisualServo 参数（从参数服务器读取，与手调 XML 一致）
        std::string vs_k_p_yaw  = std::to_string(getDbl("visual_servo/k_p_yaw_rate", 0.003));
        std::string vs_k_d_yaw  = std::to_string(getDbl("visual_servo/k_d_yaw_rate", 0.001));
        std::string vs_k_p_vz   = std::to_string(getDbl("visual_servo/k_p_vz", 0.01));
        std::string vs_k_d_vz   = std::to_string(getDbl("visual_servo/k_d_vz", 0.006));
        std::string vs_area_th  = std::to_string(getDbl("visual_servo/approach_area_threshold", 0.08));
        std::string vs_fwd_vel  = std::to_string(getDbl("visual_servo/forward_velocity", 1.0));
        std::string vs_sprint_v = std::to_string(getDbl("visual_servo/sprint_velocity", 1.5));
        std::string vs_sprint_t = std::to_string(getDbl("visual_servo/sprint_time", 1.2));
        std::string vs_initial_wait = std::to_string(getDbl("visual_servo/initial_wait_s", 2.0));
        std::string vs_lost_recovery = std::to_string(getDbl("visual_servo/lost_recovery_s", 1.5));
        std::string vs_scan_period = std::to_string(getDbl("visual_servo/scan_period_s", 4.0));
        std::string vs_scan_cycles = std::to_string(getInt("visual_servo/scan_max_cycles", 3));
        std::string vs_recovery_timeout = std::to_string(getDbl("visual_servo/recovery_timeout_s", 5.0));
        std::string vs_max_task = std::to_string(getDbl("visual_servo/max_task_time_s", 50.0));
        
        ROS_INFO("Hit balloon: color=%s -> zone=%s, approach=%s, goto=%s, yaw=%s, target=%s", 
                 color.c_str(), zone.c_str(), approach_pos.c_str(), goto_pos.c_str(), yaw.c_str(), balloon_name.c_str());
        
        return
            "      <!-- 规划飞到气球打击区域" + zone + " -->\n"
            "      <PlanningControl goal=\"" + approach_pos + "\"/>\n"
            "      <!-- 转向朝向" + zone + "区气球(" + color + ") -->\n"
            "      <GoTo goal=\"" + goto_pos + "\" yaw=\"" + yaw + "\"/>\n"
            "      <!-- 跟踪并撞击气球 -->\n"
            "      <ForceSuccess>\n"
            "        <Parallel success_count=\"1\" failure_count=\"1\">\n"
            "            <TargetTracker object_name=\"" + balloon_name + "\" "
            "confidence_threshold=\"0.8\" pixel_target=\"{balloon_pixel}\"/>\n"
            "            <VisualServo pixel_target=\"{balloon_pixel}\" result_label=\"" + cmd + "\" "
            "k_p_yaw_rate=\"" + vs_k_p_yaw + "\" k_d_yaw_rate=\"" + vs_k_d_yaw + "\" "
            "k_p_vz=\"" + vs_k_p_vz + "\" k_d_vz=\"" + vs_k_d_vz + "\" "
            "approach_area_threshold=\"" + vs_area_th + "\" "
            "forward_velocity=\"" + vs_fwd_vel + "\" sprint_velocity=\"" + vs_sprint_v + "\" sprint_time=\"" + vs_sprint_t + "\" "
            "initial_wait_s=\"" + vs_initial_wait + "\" lost_recovery_s=\"" + vs_lost_recovery + "\" "
            "scan_period_s=\"" + vs_scan_period + "\" scan_max_cycles=\"" + vs_scan_cycles + "\" "
            "recovery_timeout_s=\"" + vs_recovery_timeout + "\" max_task_time_s=\"" + vs_max_task + "\"/>\n"
            "        </Parallel>\n"
            "      </ForceSuccess>\n";
    }
    
    // ---- land ----
    if (cmd == "land") {
        std::string ret_pos = getStr("land/return_pos", "0.0;0.0;0.5");
        double ret_yaw = getDbl("land/return_yaw", 0.0);
        return
            "      <!-- 返回起点 -->\n"
            "      <GoTo goal=\"" + ret_pos + "\" yaw=\"" + std::to_string(ret_yaw) + "\" "
            "allow_outside_flight_boundary=\"true\"/>\n"
            "      <!-- 降落 -->\n"
            "      <Land/>\n";
    }
    
    ROS_WARN("Unknown LLM command: '%s', skipping.", cmd.c_str());
    return "";
}

/**
 * @brief 将 LLM 指令列表构建为完整的行为树 XML
 */
std::string buildXmlFromCommands(const std::vector<std::string>& commands) {
    std::string xml = R"(<?xml version="1.0" encoding="UTF-8"?>
<root BTCPP_format="4" main_tree_to_execute="LLMTask">
  <BehaviorTree ID="LLMTask">
    <Sequence name="LLMSequence">
)";
    for (const auto& cmd : commands) {
        std::string node_xml = commandToXmlNode(cmd);
        if (!node_xml.empty()) {
            xml += node_xml;
        }
    }
    xml += R"(    </Sequence>
  </BehaviorTree>
</root>)";
    return xml;
}

/**
 * @brief 从 LLM 发送的原始字符串中解析指令列表
 * 
 * 支持两种格式：
 *   1. 纯文本，每行一条指令
 *   2. JSON: {"commands": ["takeoff", "cross_frame", ...]}
 */
std::vector<std::string> parseCommands(const std::string& raw) {
    std::vector<std::string> commands;
    
    // 有效 JSON 只接受字符串数组；格式错误时不退回文本模式误执行。
    auto j = json::parse(raw, nullptr, false);
    if (!j.is_discarded()) {
        if (!j.is_object() || !j.contains("commands") || !j["commands"].is_array()) {
            return {};
        }
        for (const auto& item : j["commands"]) {
            if (!item.is_string()) return {};
            commands.push_back(item.get<std::string>());
        }
        return commands;
    }
    
    // 纯文本模式：按行分割
    std::istringstream stream(raw);
    std::string line;
    while (std::getline(stream, line)) {
        // 去除空白
        line.erase(0, line.find_first_not_of(" \t\r\n"));
        line.erase(line.find_last_not_of(" \t\r\n") + 1);
        if (!line.empty()) {
            commands.push_back(line);
        }
    }
    
    return commands;
}

bool isSupportedCommand(const std::string& cmd) {
    return cmd == "takeoff" || cmd == "cross_frame" ||
           cmd == "detect_aruco" || cmd == "return_cross_frame" ||
           cmd == "land" || cmd == "hit_balloon:red" ||
           cmd == "hit_balloon:purple" || cmd == "hit_balloon:blue";
}

bool validateBalloonStations(ros::NodeHandle& nh) {
    std::array<std::array<double, 2>, 3> positions{};
    constexpr std::array<const char*, 3> zones{{"A", "B", "C"}};
    for (size_t i = 0; i < zones.size(); ++i) {
        const std::string key = std::string("/referee_policy/balloon_zone_") + zones[i] + "/approach_pos";
        std::string value;
        std::string goal;
        double yaw;
        double x, y, z;
        char extra;
        if (!nh.getParam(key, value) || std::sscanf(value.c_str(), "%lf;%lf;%lf%c", &x, &y, &z, &extra) != 3
                || !std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)
                || !nh.getParam(std::string("/referee_policy/balloon_zone_") + zones[i] + "/goal", goal)
                || !nh.getParam(std::string("/referee_policy/balloon_zone_") + zones[i] + "/yaw", yaw)
                || !std::isfinite(yaw)) {
            ROS_ERROR("Invalid or missing balloon station: %s", key.c_str());
            return false;
        }
        double gx, gy, gz;
        if (std::sscanf(goal.c_str(), "%lf;%lf;%lf%c", &gx, &gy, &gz, &extra) != 3
                || !std::isfinite(gx) || !std::isfinite(gy) || !std::isfinite(gz)) {
            ROS_ERROR("Invalid balloon station goal in zone %s", zones[i]);
            return false;
        }
        positions[i] = {x, y};
    }
    for (size_t i = 0; i < positions.size(); ++i) {
        for (size_t j = i + 1; j < positions.size(); ++j) {
            const double distance = std::hypot(positions[i][0] - positions[j][0],
                                               positions[i][1] - positions[j][1]);
            if (distance < 0.6) {
                ROS_ERROR("Balloon stations %s/%s are only %.3fm apart; require >= 0.6m",
                          zones[i], zones[j], distance);
                return false;
            }
        }
    }
    for (const char* color : {"red", "purple", "blue"}) {
        std::string mapped_zone;
        if (!nh.getParam(std::string("/balloon_color_") + color, mapped_zone)
                || (mapped_zone != "A" && mapped_zone != "B" && mapped_zone != "C")) {
            ROS_ERROR("Invalid balloon color-to-zone mapping for %s", color);
            return false;
        }
    }
    ROS_INFO("Balloon station XY distances validated (>= 0.6m)");
    return true;
}

//==============================================================================
// LLM 模式全局状态
//==============================================================================

static BehaviorTreeFactory* g_factory_ptr = nullptr;
static ros::Publisher* g_result_pub_ptr = nullptr;
static std::mutex g_execution_mutex;
static std::atomic<bool> g_abort_requested{false};

/**
 * @brief LLM 指令回调 - 动态构建并执行行为树
 */
void llmCommandCallback(const std_msgs::String::ConstPtr& msg) {
    // 防止并发执行
    std::unique_lock<std::mutex> lock(g_execution_mutex, std::try_to_lock);
    if (!lock.owns_lock()) {
        ROS_WARN("Mission already in progress, ignoring new command.");
        if (g_result_pub_ptr) {
            std_msgs::String result;
            result.data = "BUSY";
            g_result_pub_ptr->publish(result);
        }
        return;
    }
    
    g_abort_requested = false;
    ros::param::set("/mission_runtime/balloon_stage_failed", false);
    ROS_INFO("=== Received LLM Command ===");
    ROS_INFO("Raw: %s", msg->data.c_str());
    
    // 1. 解析指令
    auto commands = parseCommands(msg->data);
    if (commands.empty()) {
        ROS_ERROR("No valid commands parsed from input.");
        if (g_result_pub_ptr) {
            std_msgs::String result;
            result.data = "ERROR:NO_COMMANDS";
            g_result_pub_ptr->publish(result);
        }
        return;
    }
    for (const auto& cmd : commands) {
        if (!isSupportedCommand(cmd)) {
            ROS_ERROR("Unsupported LLM command: '%s'; rejecting whole task.", cmd.c_str());
            if (g_result_pub_ptr) {
                std_msgs::String result;
                result.data = "ERROR:INVALID_COMMAND";
                g_result_pub_ptr->publish(result);
            }
            return;
        }
    }
    const bool has_balloon_task = std::any_of(
        commands.begin(), commands.end(),
        [](const std::string& cmd) { return cmd.rfind("hit_balloon:", 0) == 0; });
    
    ROS_INFO("Parsed %zu command(s):", commands.size());
    for (size_t i = 0; i < commands.size(); ++i) {
        ROS_INFO("  [%zu] %s", i, commands[i].c_str());
    }
    
    // 2. 构建 XML
    std::string xml = buildXmlFromCommands(commands);
    ROS_INFO("Generated BT XML:\n%s", xml.c_str());
    
    // 3. 创建并执行行为树
    try {
        auto tree = g_factory_ptr->createTreeFromText(xml);
        ROS_INFO("=== Starting LLM Mission Execution ===");
        
        // 打印黑板参数
        tree.rootBlackboard()->debugMessage();
        
        StdCoutLogger logger(tree);
        
        // 执行行为树（支持中途取消）
        NodeStatus status = NodeStatus::RUNNING;
        while (status == NodeStatus::RUNNING && ros::ok()) {
            if (g_abort_requested) {
                tree.haltTree();
                ROS_WARN("=== Mission Aborted ===");
                if (g_result_pub_ptr) {
                    std_msgs::String result;
                    result.data = "ABORTED";
                    g_result_pub_ptr->publish(result);
                }
                return;
            }
            status = tree.tickOnce();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        
        ROS_INFO("=== LLM Mission Completed (status: %s) ===", 
                 toStr(status).c_str());
        
        if (g_result_pub_ptr) {
            std_msgs::String result;
            // 气球段失败会继续后续任务；最终结果仍需保留失败标记。
            bool balloon_failed = false;
            ros::param::get("/mission_runtime/balloon_stage_failed", balloon_failed);
            result.data = (status == NodeStatus::SUCCESS)
                ? (balloon_failed ? "FAILURE:BALLOON_SEGMENT" :
                   (has_balloon_task ? "FLOW_COMPLETED_BALLOON_UNVERIFIED" : "SUCCESS"))
                : "FAILURE";
            g_result_pub_ptr->publish(result);
        }
        
    } catch (const std::exception& e) {
        ROS_ERROR("Failed to execute LLM mission: %s", e.what());
        if (g_result_pub_ptr) {
            std_msgs::String result;
            result.data = std::string("ERROR:") + e.what();
            g_result_pub_ptr->publish(result);
        }
    }
}

/**
 * @brief 中止指令回调
 */
void abortCallback(const std_msgs::String::ConstPtr& msg) {
    ROS_WARN("Abort requested!");
    g_abort_requested = true;
}

//==============================================================================
// 主函数
//==============================================================================

int main(int argc, char** argv) {
    // 1. 信号处理器注册
    registerSignalHandlers();
    
    // 2. ROS 初始化
    ros::init(argc, argv, "mission");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    
    ROS_INFO("=== Behavior Tree Mission Node Started ===");
    
    // 3. 初始化 MAVROS 连接管理器
    auto mavros_connector = MavRosConnect::getInstance(
        std::make_shared<ros::NodeHandle>(nh)
    );
    
    // 4. 加载配置
    BehaviorTreeConfig config = loadConfiguration(nh_private);
    if (config.mode == "llm" && !validateBalloonStations(nh)) return 1;
    
    // 5. 加载插件
    std::vector<std::string> plugin_files = loadPluginLibraries();
    
    // 6. 创建行为树工厂并注册插件
    BehaviorTreeFactory factory;
    registerPlugins(factory, plugin_files);
    
    // 7. 如果需要生成 XML 模型文件，生成后退出
    if (config.generate_xml) {
        if (generateNodeModelXML(factory, config.xml_output_file)) {
            ROS_INFO("XML generation completed. Exiting.");
            return 0;
        } else {
            ROS_ERROR("XML generation failed.");
            return 1;
        }
    }
    
    // ========== 根据模式运行 ==========
    
    if (config.mode == "llm") {
        // ---- LLM 模式：监听 Topic 动态执行 ----
        ROS_INFO("=== Running in LLM mode ===");
        
        g_factory_ptr = &factory;
        
        // 订阅 LLM 指令 Topic
        ros::Subscriber cmd_sub = nh.subscribe("/mission_command", 1, llmCommandCallback);
        // 订阅中止 Topic
        ros::Subscriber abort_sub = nh.subscribe("/mission_abort", 1, abortCallback);
        // 发布执行结果
        ros::Publisher result_pub = nh.advertise<std_msgs::String>("/mission_result", 10);
        g_result_pub_ptr = &result_pub;
        
        ROS_INFO("Listening on /mission_command for LLM commands...");
        ROS_INFO("Send abort to /mission_abort to cancel running mission.");
        ROS_INFO("Results published to /mission_result");
        
        // 启动异步回调处理
        ros::AsyncSpinner spinner(ASYNC_SPINNER_THREADS);
        spinner.start();
        
        ros::waitForShutdown();
        spinner.stop();
        
    } else {
        // ---- Static 模式：原有行为 ----
        ROS_INFO("=== Running in Static mode ===");
        
        // 8. 从配置文件创建行为树
        Tree tree;
        try {
            tree = factory.createTreeFromFile(config.tree_config_path);
            ROS_INFO("Behavior tree loaded successfully from: %s", config.tree_config_path.c_str());
            
            // 打印黑板参数（用于调试）
            ROS_INFO("=== Blackboard Parameters ===");
            tree.rootBlackboard()->debugMessage();
            
        } catch (const std::exception& e) {
            ROS_FATAL("Failed to create behavior tree: %s", e.what());
            return 1;
        }
        
        // 9. 启用日志记录（可选）
        StdCoutLogger logger(tree);
        
        // 10. 启动异步回调处理线程
        ros::AsyncSpinner spinner(ASYNC_SPINNER_THREADS);
        spinner.start();
        ROS_INFO("AsyncSpinner started with %d threads", ASYNC_SPINNER_THREADS);
        
        // 11. 执行行为树（阻塞直到任务完成）
        ROS_INFO("=== Starting Behavior Tree Execution ===");
        tree.tickWhileRunning();
        ROS_INFO("=== Behavior Tree Execution Completed ===");
        
        // 12. 清理和退出
        spinner.stop();
    }
    
    ROS_INFO("Mission node shutting down.");
    return 0;
}
