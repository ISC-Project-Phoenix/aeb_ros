cmake_minimum_required(VERSION 3.8)
project(aeb_ros)
include(FetchContent)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

FetchContent_Declare(
        aeb_cpp
        GIT_REPOSITORY https://github.com/ISC-Project-Phoenix/aeb_cpp.git
)
FetchContent_MakeAvailable(aeb_cpp)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(aeb src/aeb.cpp src/aeb_node.cpp)
target_include_directories(aeb PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
target_compile_features(aeb PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(aeb aeb_cpp)

ament_target_dependencies(
        aeb
        rclcpp
        std_msgs
        geometry_msgs
        tf2_ros
        nav_msgs
        sensor_msgs
)

install(TARGETS aeb
        DESTINATION lib/${PROJECT_NAME})

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # comment the line when a copyright and license is added to all source files
    set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # comment the line when this package is in a git repo and when
    # a copyright and license is added to all source files
    #set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif ()

ament_package()
