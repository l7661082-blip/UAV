cmake_minimum_required(VERSION 3.0.2)
project(mission_pkg)

add_compile_options(-std=c++17 -g -rdynamic)

find_package(catkin REQUIRED
    roscpp
    std_msgs
    std_srvs
    mavros_msgs
    behaviortree_cpp
    geometry_msgs
    common_msgs
    message_filters
    tf2
    tf2_geometry_msgs
    quadrotor_msgs
    sensor_msgs
    nav_msgs
    visualization_msgs
    cv_bridge
    )
find_package(OpenCV REQUIRED)

catkin_package(
  CATKIN_DEPENDS behaviortree_cpp
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${OpenCV_INCLUDE_DIRS}
)

add_library(mavros_cnt SHARED src/mavros_cnt.cpp)
target_link_libraries(mavros_cnt PRIVATE ${catkin_LIBRARIES})

# 基类库
add_library(drone_base SHARED 
    src/base/drone_action_node.cpp
    src/utils/pose_utils.cpp
)
target_link_libraries(drone_base PRIVATE mavros_cnt behaviortree_cpp ${catkin_LIBRARIES})

add_library(target_det_base SHARED 
    src/base/target_detector_base.cpp
)
target_link_libraries(target_det_base PRIVATE mavros_cnt drone_base behaviortree_cpp ${catkin_LIBRARIES})

list(APPEND plugin_libs mavros_cnt drone_base target_det_base)

# 扫描所有节点子目录下的 cpp 进行插件化构建
set(LIBSOURCE src/nodes)
file(GLOB_RECURSE SOURCE_FILES 
    "${LIBSOURCE}/action/*.cpp" 
    "${LIBSOURCE}/perception/*.cpp"
    "${LIBSOURCE}/condition/*.cpp" 
    "${LIBSOURCE}/control/*.cpp" 
    "${LIBSOURCE}/decorator/*.cpp"
)

foreach(SOURCE_FILE ${SOURCE_FILES})
    # 排除备份文件
    if(SOURCE_FILE MATCHES ".*\\.orig")
        continue()
    endif()

    get_filename_component(NODE_LIBS ${SOURCE_FILE} NAME_WE)
    add_library(${NODE_LIBS} SHARED ${SOURCE_FILE})
    target_compile_definitions(${NODE_LIBS} PRIVATE BT_PLUGIN_EXPORT)
    set_target_properties(${NODE_LIBS} PROPERTIES PREFIX "")
    target_link_libraries(${NODE_LIBS} PRIVATE ${BTCPP_LIBRARY} mavros_cnt drone_base target_det_base ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
endforeach()

add_executable(bt_ros src/bt_ros.cpp)
add_dependencies(bt_ros ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(bt_ros
   ${catkin_LIBRARIES}
    mavros_cnt
    drone_base
    dl
 )

# ArUco 独立检测节点（带服务开关）
add_executable(aruco_detect_node src/aruco_detect_node.cpp)
add_dependencies(aruco_detect_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(aruco_detect_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
)

