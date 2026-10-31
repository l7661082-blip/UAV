#!/bin/bash
###############################################################################
# sim_env_start.sh
# 仿真环境一键启动脚本（含大模型 LLM）
#
# 启动内容：
#   0.  roscore                   — ROS 核心
#   1.  LLM (mission_once.py)     — 大模型加载（耗时，最先启动）    [独立 xterm]
#   2.  detection_color.launch    — YOLO 目标检测 + HSV 颜色分类    [独立 xterm]
#   3.  MAVROS                    — 飞控通信（当前已注释）
#   4.  sensor_pkg/main.py        — 传感器 + VisionCapture
#   5.  AgentContestMap.py        — 加载比赛场景地图
#   6.  cloud_transform.launch    — 深度点云坐标转换
#   7.  aruco_detect.launch       — ArUco 二维码识别                [独立 xterm]
#   8.  ego-planner.launch        — Ego-Planner 避障规划            [独立 xterm]
#   9.  llm_mission.launch        — LLM 任务节点（监听指令）        [独立 xterm]
#  10.  Setpoint Monitor          — 监控飞控设定点                  [独立 xterm]
#  11.  ROS Node Resource Monitor — ROS 节点资源监控                [独立 xterm]
###############################################################################

# ========== 修复乱码：强制 UTF-8 ==========
export LANG=en_US.UTF-8
export LC_ALL=en_US.UTF-8
export LC_CTYPE=en_US.UTF-8
export TERM=xterm-256color
export PYTHONIOENCODING=utf-8

# 注意: 不使用 set -e，因为后台进程崩溃时不应终止整个脚本
bash ./kill_ros_pid.sh

# ========== 颜色输出 ==========
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
RED='\033[0;31m'
NC='\033[0m'

# ========== Xterm 公共参数（与 test.sh 保持一致）==========
XTERM_OPTS="-en UTF-8 -u8 -lc -sl 10000 -bg black -fg white -fa 'Monospace' -fs 12"
XTERM_ENV="export LANG=en_US.UTF-8; export LC_ALL=en_US.UTF-8; export TERM=xterm-256color; export PYTHONIOENCODING=utf-8; unset PROMPT_COMMAND"

# 获取脚本所在目录（catkin_ws）
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="${SCRIPT_DIR}"

startup_abort() {
    echo -e "${RED}[ERROR] 启动检查失败，停止已启动的后台进程${NC}"
    for pid in "${LLM_PID:-}" "${DET_PID:-}" "${SENSOR_PID:-}" \
               "${MAP_PID:-}" "${CLOUD_PID:-}" "${ARUCO_PID:-}" \
               "${EGO_PID:-}" "${ROSCORE_PID:-}"; do
        if [ -n "$pid" ]; then kill "$pid" 2>/dev/null || true; fi
    done
    exit 1
}

# ========== 配置 xterm 剪贴板粘贴 (Ctrl+Shift+V / Shift+Insert) ==========
XRESOURCES_FILE="/tmp/.xterm_clipboard_resources"
cat > "${XRESOURCES_FILE}" << 'XEOF'
XTerm*selectToClipboard: true
XTerm*allowTitleOps: false
XTerm.VT100.translations: #override \
    Ctrl Shift <Key>V: insert-selection(CLIPBOARD) \n\
    Shift <Key>Insert: insert-selection(CLIPBOARD)
XEOF
xrdb -merge "${XRESOURCES_FILE}" 2>/dev/null || true

echo -e "${CYAN}============================================${NC}"
echo -e "${CYAN}  仿真环境启动脚本 (Visual Servo Balloon)  ${NC}"
echo -e "${CYAN}============================================${NC}"
echo -e "${YELLOW}工作空间: ${WS_DIR}${NC}"

# source 工作空间
if [ -f "${WS_DIR}/devel/setup.bash" ]; then
    source "${WS_DIR}/devel/setup.bash"
    echo -e "${GREEN}[OK] Sourced devel/setup.bash${NC}"

    # --- RflySim SDK PYTHONPATH ---
    RFLYSIM_SDK="/mnt/c/PX4PSP/RflySimAPIs/RflySimSDK"
    if [ -d "${RFLYSIM_SDK}" ]; then
        export PYTHONPATH="${PYTHONPATH}:${RFLYSIM_SDK}:${RFLYSIM_SDK}/comm:${RFLYSIM_SDK}/ctrl:${RFLYSIM_SDK}/ue:${RFLYSIM_SDK}/vision:${RFLYSIM_SDK}/phm:${RFLYSIM_SDK}/swarm:${RFLYSIM_SDK}/adb:${RFLYSIM_SDK}/wsl:${RFLYSIM_SDK}/test:${RFLYSIM_SDK}/word"
        echo -e "${GREEN}[OK] Added RflySim SDK to PYTHONPATH${NC}"
    else
        echo -e "${YELLOW}[WARN] RflySim SDK not found at ${RFLYSIM_SDK}${NC}"
    fi
else
    echo -e "${RED}[ERROR] ${WS_DIR}/devel/setup.bash not found! Run catkin_make first.${NC}"
    exit 1
fi

# ===================== MAVROS 参数 =====================
MAVROS_FCU_URL="udp://:20101@"

# =====================================================================
# 0. 确保 roscore 已启动
# =====================================================================
echo ""
echo -e "${GREEN}[0/9] 确保 roscore 已启动 ...${NC}"

if ! pgrep -f rosmaster > /dev/null 2>&1; then
    echo -e "${YELLOW}      rosmaster 未运行，正在启动 roscore ...${NC}"
    roscore &
    ROSCORE_PID=$!
    echo -e "${GREEN}      roscore PID: ${ROSCORE_PID}${NC}"
    for i in $(seq 1 10); do
        if pgrep -f rosmaster > /dev/null 2>&1; then
            echo -e "${GREEN}      roscore 已就绪${NC}"
            break
        fi
        sleep 1
    done
    if ! pgrep -f rosmaster > /dev/null 2>&1; then
        echo -e "${RED}[ERROR] roscore 启动失败！${NC}"
        exit 1
    fi
else
    echo -e "${GREEN}      rosmaster 已在运行${NC}"
fi

# =====================================================================
# 1. 启动大模型 LLM（独立 xterm，模型加载耗时，最先启动）
# =====================================================================
echo ""
echo -e "${GREEN}[1/11] 启动大模型 LLM [独立 xterm] ...模型加载需要10-30秒${NC}"

xterm $XTERM_OPTS -T "LLM Mission (mission_once)" \
    -e "bash --login -c '${XTERM_ENV}; source ${WS_DIR}/devel/setup.bash && cd /home/ubuntu/model && python3 mission_once.py 2>&1; echo \"Press Enter to close\"; read; exec bash'" &
LLM_PID=$!
echo -e "${GREEN}      LLM PID: ${LLM_PID}${NC}"

# =====================================================================
# 2. 启动 YOLO 目标检测（独立 xterm，模型加载较慢，先启动）
# =====================================================================
echo ""
echo -e "${GREEN}[2/11] 启动 YOLO 目标检测 + 气球颜色分类 [独立 xterm] ...${NC}"

xterm $XTERM_OPTS -T "YOLO Detection" \
    -e "bash --login -c '${XTERM_ENV}; source ${WS_DIR}/devel/setup.bash && roslaunch object_det detection_color.launch 2>&1; echo \"Press Enter to close\"; read; exec bash'" &
DET_PID=$!
echo -e "${GREEN}      PID: ${DET_PID}${NC}"

# 模型与 YOLO 并行预热；提示词注入成功后才视为 LLM 可用。
echo -e "${YELLOW}      等待大模型就绪，最长 60s ...${NC}"
if ! timeout 60s rostopic echo -n 1 /mission_llm_ready 2>/dev/null | grep -q 'READY'; then
    echo -e "${RED}[ERROR] 大模型 60s 内未就绪，停止启动任务节点${NC}"
    startup_abort
fi
echo -e "${GREEN}[OK] 大模型与提示词已就绪${NC}"

# =====================================================================
# 3. 启动 MAVROS（当前已注释）
# =====================================================================
echo ""
echo -e "${GREEN}[3/11] 启动 MAVROS ...${NC}"
echo -e "${YELLOW}      FCU URL: ${MAVROS_FCU_URL}${NC}"

#roslaunch mavros px4.launch fcu_url:=${MAVROS_FCU_URL} &
#MAVROS_PID=$!

# =====================================================================
# 4. 启动传感器 (sensor_pkg/main.py)
# =====================================================================
echo ""
echo -e "${GREEN}[4/11] 启动传感器 (main.py) ...${NC}"
echo -e "${YELLOW}      VisionCapture + 传感器数据发布${NC}"

cd "${WS_DIR}/src/sensor_pkg"
python3 main.py &
SENSOR_PID=$!
echo -e "${GREEN}      PID: ${SENSOR_PID}${NC}"

echo -e "${YELLOW}      等待初始化 (5s)...${NC}"
sleep 5

# =====================================================================
# 5. 加载比赛场景地图 (AgentContestMap.py)
# =====================================================================
echo ""
echo -e "${GREEN}[5/11] 加载场景地图 (AgentContestMap.py) ...${NC}"

cd "${WS_DIR}/src/sensor_pkg"
python3 AgentContestMap.py &
MAP_PID=$!
echo -e "${GREEN}      PID: ${MAP_PID}${NC}"

sleep 3

# =====================================================================
# 6. 启动点云坐标转换
# =====================================================================
echo ""
echo -e "${GREEN}[6/11] 启动点云坐标转换 (cloud_transform) ...${NC}"

roslaunch sensor_pkg cloud_transform.launch &
CLOUD_PID=$!
echo -e "${GREEN}      PID: ${CLOUD_PID}${NC}"

sleep 2

# =====================================================================
# 7. 启动 ArUco 二维码识别（独立 xterm）
# =====================================================================
echo ""
echo -e "${GREEN}[7/11] 启动 ArUco 识别节点 [独立 xterm] ...${NC}"

xterm $XTERM_OPTS -T "ArUco Detect" \
    -e "bash --login -c '${XTERM_ENV}; source ${WS_DIR}/devel/setup.bash && roslaunch mission_pkg aruco_detect.launch show_debug:=true 2>&1; echo \"Press Enter to close\"; read; exec bash'" &
ARUCO_PID=$!
echo -e "${GREEN}      PID: ${ARUCO_PID}${NC}"

sleep 2

# =====================================================================
# 8. 启动 Ego-Planner（独立 xterm）
# =====================================================================
echo ""
echo -e "${GREEN}[8/11] 启动 Ego-Planner [独立 xterm] ...${NC}"

xterm $XTERM_OPTS -T "Ego-Planner" \
    -e "bash --login -c '${XTERM_ENV}; source ${WS_DIR}/devel/setup.bash && roslaunch ego_planner FS-J310_ego-planner.launch 2>&1; echo \"Press Enter to close\"; read; exec bash'" &
EGO_PID=$!
echo -e "${GREEN}      PID: ${EGO_PID}${NC}"

sleep 2

# =====================================================================
# 等待 YOLO 模型加载完成
# =====================================================================
echo ""
echo -e "${YELLOW}等待 YOLO 模型加载完成 ...${NC}"
if ! timeout 120s rostopic echo -n 1 /objects >/dev/null 2>&1; then
    echo -e "${RED}[ERROR] 120s 内未收到 /objects 检测结果，请检查 YOLO 和图像话题${NC}"
    startup_abort
fi
echo -e "${GREEN}[OK] /objects 已收到检测结果${NC}"

# =====================================================================
# 9. 启动 LLM 任务节点（独立 xterm，监听 /mission_command）
# =====================================================================
echo ""
echo -e "${GREEN}[9/11] 启动 LLM 任务节点 [独立 xterm] ...${NC}"

mkdir -p /tmp/mission_logs
# 待裁判口径确认：以下评分参数仅供记录，任务节点不据此自动判分。
rosparam load "${WS_DIR}/src/mission_pkg/config/referee_policy.yaml" /referee_policy

xterm $XTERM_OPTS -T "Mission (llm_mission)" \
    -e "bash --login -c '${XTERM_ENV}; source ${WS_DIR}/devel/setup.bash && roslaunch mission_pkg llm_mission.launch 2>&1; echo \"Press Enter to close\"; read; exec bash'" &
MISSION_PID=$!
echo -e "${GREEN}      PID: ${MISSION_PID}${NC}"

sleep 2

# =====================================================================
# 9. 监控 setpoint_raw/local（独立 xterm）
# =====================================================================
echo ""
echo -e "${GREEN}[10/11] 启动 Setpoint Monitor [独立 xterm] ...${NC}"

xterm $XTERM_OPTS -T "Setpoint Monitor" \
    -e "bash --login -c '${XTERM_ENV}; source ${WS_DIR}/devel/setup.bash && rostopic echo /mavros/setpoint_raw/local 2>&1; echo \"Press Enter to close\"; read; exec bash'" &
SETPOINT_MON_PID=$!
echo -e "${GREEN}      Setpoint Monitor PID: ${SETPOINT_MON_PID}${NC}"

# =====================================================================
# 10. 系统资源监控（独立 xterm）
# =====================================================================
echo ""
echo -e "${GREEN}[11/11] 启动 ROS 节点资源监控 [独立 xterm] ...${NC}"

xterm $XTERM_OPTS -T "ROS Node Resource Monitor" \
    -e "bash --login -c '${XTERM_ENV}; watch -n 2 \"echo ===== ROS Node Resource Usage ===== && ps -eo pid,pcpu,pmem,rss,comm,args --sort=-pcpu | head -1 && ps -eo pid,pcpu,pmem,rss,comm,args --sort=-pcpu | grep -E \\\"ros|mavros|ego_planner|det_color|aruco|bt_ros|main.py|python3|cloud_transform\\\" | grep -v grep | head -20\"'" &
SYS_MON_PID=$!
echo -e "${GREEN}      ROS Node Monitor PID: ${SYS_MON_PID}${NC}"

echo ""
echo -e "${CYAN}============================================${NC}"
echo -e "${CYAN}  所有节点已启动！${NC}"
echo -e "${CYAN}============================================${NC}"
echo ""
echo -e "${YELLOW}>>> 在其他终端中发送指令 <<<${NC}"
echo ""
echo -e "  ${GREEN}# 发送指令：${NC}"
echo -e "  # 起飞 + 穿框 + 降落"
echo -e '  rostopic pub /mission_command std_msgs/String "data: '"'"'{"commands":["takeoff","cross_frame","land"]}'"'"'" --once'
echo ""
echo -e "  # 起飞 + 刺红色气球 + 降落"
echo -e '  rostopic pub /mission_command std_msgs/String "data: '"'"'{"commands":["takeoff","hit_balloon:red","land"]}'"'"'" --once'
echo ""
echo -e "  # 中止当前任务"
echo -e '  rostopic pub /mission_abort std_msgs/String "data: '"'"'abort'"'"'" --once'
echo ""
echo -e "  # 查看执行结果"
echo -e "  rostopic echo /mission_result"
echo ""
echo -e "${CYAN}============================================${NC}"
echo -e "${YELLOW}按 Ctrl+C 关闭所有后台进程${NC}"

# Ctrl+C 清理
cleanup() {
    echo ""
    echo -e "${RED}正在关闭所有进程...${NC}"
    kill $ROSCORE_PID $LLM_PID $MAVROS_PID $SENSOR_PID $MAP_PID $CLOUD_PID $DET_PID $ARUCO_PID $EGO_PID $MISSION_PID $SETPOINT_MON_PID $SYS_MON_PID 2>/dev/null
    pkill -f "roscore" 2>/dev/null
    pkill -f "mavros" 2>/dev/null
    pkill -f "cloud_transform" 2>/dev/null
    pkill -f "det_color" 2>/dev/null
    pkill -f "ego_planner" 2>/dev/null
    pkill -f "bt_ros" 2>/dev/null
    pkill -f "mission_once" 2>/dev/null
    echo -e "${GREEN}所有进程已关闭。${NC}"
    exit 0
}
trap cleanup SIGINT SIGTERM

# 保持前台运行
wait
