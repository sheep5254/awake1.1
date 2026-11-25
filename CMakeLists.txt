cmake_minimum_required(VERSION 3.14)
cmake_policy(SET CMP0072 NEW)
project(awakening LANGUAGES CXX C)


set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_BUILD_TYPE "Release")
message(STATUS "--------------------CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}--------------------")

#修改以下option需要rebuild清除缓存才能生效
option(BUILD_WITH_TRT      "Enable TensorRT backend"   ON)
option(BUILD_WITH_OPENVINO "Enable OpenVINO backend"   ON)
option(BUILD_SENTRY "Enable Sentry"   ON)
option(BUILD_RADAR "Enable Radar"   ON)
option(BUILD_RUNTIME_TESTS "Build runtime test executables by default" ON)
option(BUILD_WITH_PCH "Enable precompiled headers for C++ targets" ON)
option(BUILD_WITH_Daedalus "Enable Daedalus interface" ON)
option(BUILD_WITH_RERUN "Enable Rerun visualization when the SDK is available" ON)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake)

# Daedalus 模拟器接口使用 std::expected，需要支持 C++23 标准库的编译器（GCC 13+ / libstdc++ 13+）
if(BUILD_WITH_Daedalus)
    include(CheckCXXSourceCompiles)
    check_cxx_source_compiles("
        #include <expected>
        int main() { std::expected<int, int> e{1}; return *e; }
    " AWAKENING_HAS_STD_EXPECTED)
    if(NOT AWAKENING_HAS_STD_EXPECTED)
        message(WARNING "std::expected is not available with the current compiler; "
                        "disabling the Daedalus simulation interface (needs GCC 13+ / libstdc++ 13+)")
        set(BUILD_WITH_Daedalus OFF)
    endif()
endif()

set(AWAKENING_PCH_HEADER ${PROJECT_SOURCE_DIR}/src/pch.hpp)
include(AwakeningTargets)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(fmt REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Ceres REQUIRED)
find_package(spdlog REQUIRED)
find_package(Boost REQUIRED)
find_package(TBB REQUIRED)
find_package(HikSDK QUIET)
find_package(MvSDK QUIET)
set(CMAKE_PREFIX_PATH "${CMAKE_PREFIX_PATH};/home/hy/Galaxy_Linux-x86_Gige-U3_32bits-64bits_2.6.2606.9251/Galaxy_camera")
find_package(DahengSDK QUIET)
set(BUILD_DEFINITIONS "")
set(BUILD_LIBS "")
if(HikSDK_FOUND)
    list(APPEND BUILD_DEFINITIONS USE_HikSDK)
    list(APPEND BUILD_LIBS HikSDK::HikSDK)
endif()

if(MvSDK_FOUND)
    list(APPEND BUILD_DEFINITIONS USE_MvSDK)
    list(APPEND BUILD_LIBS MvSDK::MvSDK)
endif()

if(DahengSDK_FOUND)
    list(APPEND BUILD_DEFINITIONS USE_DahengSDK)
    list(APPEND BUILD_LIBS DahengSDK::DahengSDK)
endif()

set(RERUN_FOUND FALSE)
if(BUILD_WITH_RERUN)
    find_package(rerun_sdk CONFIG QUIET)
    if(TARGET rerun_sdk)
        set(RERUN_FOUND TRUE)
        message(STATUS "Rerun SDK found, enabling the Rerun visualization backend")
    else()
        message(WARNING "Rerun SDK not found; Web/ROS2 visualization remains available")
    endif()
endif()



if(RERUN_FOUND)
    list(APPEND BUILD_DEFINITIONS USE_RERUN)
endif()
if(BUILD_WITH_Daedalus)
    list(APPEND BUILD_DEFINITIONS USE_Daedalus)
endif()

if(BUILD_WITH_OPENVINO)
    find_package(OpenVINO REQUIRED COMPONENTS Runtime ONNX)
    list(APPEND BUILD_DEFINITIONS USE_OPENVINO)
    list(APPEND BUILD_LIBS openvino::runtime openvino::frontend::onnx)
endif()

if(BUILD_WITH_TRT)
    find_package(CUDAToolkit QUIET)
    set(CMAKE_PREFIX_PATH "${CMAKE_PREFIX_PATH};/home/hy/TensorRT-10.6.0.26")
    find_package(TensorRT QUIET)
endif()

if(BUILD_WITH_TRT AND CUDAToolkit_FOUND AND TensorRT_FOUND)
    include_directories(${TensorRT_INCLUDE_DIR})
    include_directories(/usr/local/cuda/include)
    set(CMAKE_CUDA_COMPILER /usr/local/cuda/bin/nvcc)
    set(CMAKE_CUDA_ARCHITECTURES 86)
    enable_language(CUDA)
    set(CMAKE_CUDA_STANDARD 17)
    set(CMAKE_CUDA_STANDARD_REQUIRED ON)

    list(APPEND BUILD_DEFINITIONS USE_TRT)
    list(APPEND BUILD_LIBS TensorRT::TensorRT TensorRT::nvonnxparser CUDA::cudart  /usr/local/cuda/lib64/libnppicc.so)
elseif(BUILD_WITH_TRT)
    message(WARNING "CUDA Toolkit or TensorRT not found (CUDA: ${CUDAToolkit_FOUND}, TensorRT: ${TensorRT_FOUND}); "
                    "TensorRT backend disabled, using OpenVINO only")
endif()

configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/src/param_deliver.h.in  
    ${CMAKE_BINARY_DIR}/src/param_deliver.h            
    @ONLY
)
awakening_add_library(relink SOURCES src/relink.cpp)
include_directories(${PROJECT_NAME}_utils PUBLIC
    ${PROJECT_SOURCE_DIR}/src
    ${PROJECT_SOURCE_DIR}/3rdparty
    ${CMAKE_BINARY_DIR}/src
    ${OpenCV_INCLUDE_DIRS}
)
set(UTILS_SOURCE_GLOBS src/utils/*.cpp 3rdparty/*.cpp 3rdparty/*.c)
if(BUILD_WITH_TRT)
    list(APPEND UTILS_SOURCE_GLOBS src/utils/*.cu)
endif()
awakening_add_library(${PROJECT_NAME}_utils GLOB ${UTILS_SOURCE_GLOBS} DEFINITIONS ${BUILD_DEFINITIONS} LINKS
    HikSDK::HikSDK
    MvSDK::MvSDK
    yaml-cpp
    fmt::fmt
    Eigen3::Eigen
    Ceres::ceres
    spdlog::spdlog
    TBB::tbb
    Boost::boost
    ${OpenCV_LIBS}
    ${BUILD_LIBS}
)
awakening_add_library(${PROJECT_NAME}_auto_aim GLOB src/tasks/auto_aim/*.cpp LINKS ${PROJECT_NAME}_utils)
awakening_add_library(${PROJECT_NAME}_auto_buff GLOB src/tasks/auto_buff/*.cpp LINKS ${PROJECT_NAME}_utils)
awakening_add_library(${PROJECT_NAME}_web SOURCES src/tasks/base/web.cpp LINKS
    ${PROJECT_NAME}_auto_aim
    ${PROJECT_NAME}_auto_buff
)
awakening_add_library(${PROJECT_NAME}_vslam GLOB src/tasks/vslam/*.cpp)

if(RERUN_FOUND)
    # The distributed C++ SDK is static/non-PIC, so keep it at executable link scope.
    awakening_add_library(${PROJECT_NAME}_rerun TYPE STATIC SOURCES src/_rerun/recorder.cpp DEFINITIONS USE_RERUN
        LINKS rerun_sdk
        PRIVATE_LINKS ${PROJECT_NAME}_auto_aim ${PROJECT_NAME}_auto_buff
    )
endif()

set(ROS2_PACKAGES
    ament_cmake
    rmw
    rosidl_typesupport_cpp
    rosidl_typesupport_interface
    rclcpp
    geometry_msgs
    tf2_ros
    tf2_geometry_msgs
    sensor_msgs
    visualization_msgs
    nav_msgs
    
)
foreach(pkg IN LISTS ROS2_PACKAGES)
    find_package(${pkg} QUIET)
endforeach()


set(ROS2_FULL_FOUND TRUE)
foreach(pkg IN LISTS ROS2_PACKAGES)
    if(NOT ${pkg}_FOUND)
        set(ROS2_FULL_FOUND FALSE)
        message(WARNING "ROS2 package ${pkg} not found")
    endif()
endforeach()

if(ROS2_FULL_FOUND)
    message(STATUS "ROS2 full environment found, compiling ROS2 targets ...")
    list(APPEND BUILD_DEFINITIONS USE_ROS2)

    set(ROS2_INCLUDES)
    foreach(pkg IN LISTS ROS2_PACKAGES)
        if(TARGET ${pkg}::${pkg})
            list(APPEND ROS2_INCLUDES ${${pkg}_INCLUDE_DIRS})
        endif()
    endforeach()
    include_directories(${ROS2_INCLUDES})

    # 接法 A（视觉发话题、导航转串口）：example_interfaces 是 ROS2 自带，
    # auto_aim_interfaces 由本仓库 interfaces/ 目录提供（需 colcon 编译并 source）。
    find_package(example_interfaces QUIET)
    if(example_interfaces_FOUND)
        include_directories(${example_interfaces_INCLUDE_DIRS})
    else()
        message(WARNING "example_interfaces not found; cmd_gimbal_joint/cmd_shoot will be disabled")
    endif()
    find_package(auto_aim_interfaces QUIET)
    if(auto_aim_interfaces_FOUND)
        include_directories(${auto_aim_interfaces_INCLUDE_DIRS})
        message(STATUS "auto_aim_interfaces found, tracker/target will be published")
    else()
        message(WARNING "auto_aim_interfaces not found; tracker/target will be disabled")
    endif()
    awakening_add_library(${PROJECT_NAME}_rcl EXCLUDE_FROM_ALL GLOB src/_rcl/*.cpp LINKS ${PROJECT_NAME}_utils)
    ament_target_dependencies(${PROJECT_NAME}_rcl  ${ROS2_PACKAGES})
    target_compile_definitions(${PROJECT_NAME}_rcl  PUBLIC ${BUILD_DEFINITIONS})
    if(BUILD_SENTRY)
    awakening_add_library(${PROJECT_NAME}_sentry_brain EXCLUDE_FROM_ALL GLOB src/tasks/sentry_brain/*.cpp LINKS
        ${PROJECT_NAME}_rcl
        ${PROJECT_NAME}_auto_aim 
    )
    endif()
    if(BUILD_RADAR)
    # find_package(small_gicp REQUIRED)
    find_package(Open3D QUIET)
    if(NOT Open3D_FOUND)
        message(WARNING "Open3D not found; radar targets disabled")
        set(BUILD_RADAR OFF)
    endif()
    endif()
    if(BUILD_RADAR)
    awakening_add_library(${PROJECT_NAME}_radar_detect EXCLUDE_FROM_ALL GLOB src/tasks/radar_detect/*.cpp LINKS
        ${PROJECT_NAME}_utils
        ${PROJECT_NAME}_rcl
        Open3D::Open3D
    )
    awakening_add_library(${PROJECT_NAME}_radar_io EXCLUDE_FROM_ALL GLOB src/tasks/radar_io/*.cpp LINKS
        ${PROJECT_NAME}_utils
        ${PROJECT_NAME}_rcl
    )
    endif()
else()
    message(WARNING "ROS2 dependencies incomplete, skipping ROS2 targets ...")
endif()


set(STANDARD_LINKS ${PROJECT_NAME}_auto_aim ${PROJECT_NAME}_auto_buff ${PROJECT_NAME}_web)
set(VO_LINKS ${PROJECT_NAME}_vslam)
if(ROS2_FULL_FOUND)
    list(APPEND STANDARD_LINKS ${PROJECT_NAME}_rcl)
    list(APPEND VO_LINKS ${PROJECT_NAME}_rcl)
endif()

awakening_add_executable(standard SOURCES src/runtime/standard.cpp LINKS ${STANDARD_LINKS})

if(ROS2_FULL_FOUND)
    if(example_interfaces_FOUND)
        ament_target_dependencies(standard rclcpp sensor_msgs example_interfaces)
        target_compile_definitions(standard PRIVATE USE_NAV_CMD_PUB)
    endif()
    if(auto_aim_interfaces_FOUND)
        ament_target_dependencies(standard auto_aim_interfaces)
        target_compile_definitions(standard PRIVATE USE_NAV_TARGET_PUB)
    endif()
endif()

awakening_add_executable(control_test TOOL SOURCES src/runtime/test/control_test.cpp LINKS ${PROJECT_NAME}_web)
awakening_add_executable(delay_test TOOL SOURCES src/runtime/test/delay_test.cpp)
awakening_add_executable(vo SOURCES src/runtime/vo.cpp LINKS ${VO_LINKS})

if(BUILD_RADAR)
    awakening_add_executable(radar_detect TOOL SOURCES src/runtime/radar_detect.cpp LINKS ${PROJECT_NAME}_radar_detect ${PROJECT_NAME}_radar_io)
    awakening_add_executable(livox_v1_driver TOOL SOURCES src/runtime/livox_v1_driver.cpp LINKS ${PROJECT_NAME}_radar_detect)
    awakening_add_executable(radar_cal TOOL SOURCES src/runtime/radar_cal.cpp LINKS ${PROJECT_NAME}_radar_detect)
endif()

if(ROS2_FULL_FOUND AND BUILD_SENTRY)
    awakening_add_executable(nav TOOL SOURCES src/runtime/nav.cpp LINKS ${PROJECT_NAME}_rcl ${PROJECT_NAME}_utils)
    awakening_add_executable(sentry SOURCES src/runtime/sentry.cpp LINKS
        ${PROJECT_NAME}_rcl
        ${PROJECT_NAME}_utils
        ${PROJECT_NAME}_auto_aim
        ${PROJECT_NAME}_auto_buff
        ${PROJECT_NAME}_web
        ${PROJECT_NAME}_sentry_brain
    )
    awakening_add_executable(map_edit TOOL SOURCES src/runtime/map_edit.cpp)
endif()
