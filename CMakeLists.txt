cmake_minimum_required(VERSION 3.5)

set(PROJECT_NAME ControllerTest)
project(${PROJECT_NAME} VERSION 0.0.1 LANGUAGES CXX)


# Set C++ Standard
set(CMAKE_CXX_STANDARD 17) # Use C++17
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

option(CHECK "Check the relevant dependency libraries " TRUE)

# Eigen3
find_package(Eigen3 REQUIRED)
find_package(OpenCV  REQUIRED)

if (CHECK)
   message(STATUS "OpenCV_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS}")
   message(STATUS "Eigen_INCLUDE_DIRS: ${Eigen_INCLUDE_DIRS}")
   message(STATUS "Eigen version: ${EIGEN_VERSION}")
endif(CHECK)

# bin lib
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# third_libs json
set(THIRD_LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_libs/)
FILE(GLOB JSON_LIBRARIES ${THIRD_LIB_DIR}json/*.a)
# check
if (CHECK)
   message("THIRD_LIB_DIR: ${THIRD_LIB_DIR}")
   message("JSON_LIBRARIES: ${JSON_LIBRARIES}")
endif(CHECK)

# Set Path
set(CMAKE_COMMON_PATH
   "${CMAKE_CURRENT_SOURCE_DIR}/common/")
set(CMAKE_COMMON_MSG_PATH
   "${CMAKE_CURRENT_SOURCE_DIR}/common_msg/")
set(CMAKE_CONTROL_COMPONENT_PATH
   "${CMAKE_CURRENT_SOURCE_DIR}/control_component/")
set(CMAKE_CONTROLLER_PATH
   "${CMAKE_CURRENT_SOURCE_DIR}/controller/")

# include files
include_directories(${PROJECT_NAME}
   ${CMAKE_COMMON_PATH}/filters
   ${CMAKE_COMMON_PATH}/math
   ${CMAKE_COMMON_PATH}/data
   ${CMAKE_COMMON_PATH}/configs
   ${CMAKE_COMMON_PATH}/vehicle_state
   ${CMAKE_COMMON_MSG_PATH}/basic_msgs
   ${CMAKE_COMMON_MSG_PATH}/chassis_msgs
   ${CMAKE_COMMON_MSG_PATH}/config_msgs
   ${CMAKE_COMMON_MSG_PATH}/localization_msgs
   ${CMAKE_COMMON_MSG_PATH}/map_msgs
   ${CMAKE_COMMON_MSG_PATH}/planning_msgs
   ${CMAKE_COMMON_MSG_PATH}/routing_msgs
   ${CMAKE_CONTROL_COMPONENT_PATH}/controller_task_base/common
   ${CMAKE_CONTROL_COMPONENT_PATH}/config
   ${CMAKE_CONTROLLER_PATH}/lat_based_lqr_controller
   ${CMAKE_CONTROLLER_PATH}/lon_based_pid_controller
   ${THIRD_LIB_DIR}/json/include
   ${EIGEN_INCLUDE_DIR}
   ${OpenCV_INCLUDE_DIRS}
)


# source files
set(COMMON_CPP_FILES
   common/filters/mean_filter.cpp
   common/filters/digital_filter.cpp
   common/filters/math.cpp
   common/filters/digital_filter_coefficients.cpp
   common/math/interpolation_2d.cpp
   common/math/bspline_1d.cpp
   common/math/interpolation_1d.cpp
   common/math/linear_quadratic_regulator.cpp
   common/vehicle_state/vehicle_state_provider.cpp
   common/configs/vehicle_config_helper.cpp
   control_component/controller_task_base/common/trajectory_analyzer.cpp
   control_component/controller_task_base/common/leadlag_controller.cpp
)

set(LATCONTROLLERTEST_SOURCES
   ${COMMON_CPP_FILES}
   controller/lat_based_lqr_controller/lat_controller_test.cpp
   controller/lat_based_lqr_controller/connect_lat_controller_test.cpp
   controller/lat_based_lqr_controller/main.cpp
)

set(LONCONTROLLERTEST_SOURCES
   ${COMMON_CPP_FILES}
   controller/lon_based_pid_controller/lon_controller_test.cpp
   controller/lon_based_pid_controller/connect_lon_controller_test.cpp
   controller/lon_based_pid_controller/main.cpp
)

# add_executable
add_executable(LatControllerTest ${LATCONTROLLERTEST_SOURCES})
add_executable(LonControllerTest ${LONCONTROLLERTEST_SOURCES})

# target_link_libraries
if(LINUX)
   target_link_libraries(LatControllerTest ${JSON_LIBRARIES} ${OpenCV_LIBS} ${EIGEN_LIBRARIES})
   target_link_libraries(LonControllerTest ${JSON_LIBRARIES} ${OpenCV_LIBS} ${EIGEN_LIBRARIES})
endif()

