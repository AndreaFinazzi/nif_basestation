cmake_minimum_required(VERSION 3.5)
project(nif_msgs)

# Default to C99
if (NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif ()

# Default to C++14
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(rosidl_default_generators REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

set(msg_files
    "msg/AutonomyStatus.msg"
    "msg/ControlCommand.msg"
    "msg/FilterOut.msg"
    "msg/SystemHealthStatus.msg"
    "msg/Perception3D.msg"
    "msg/Perception3DArray.msg"
    "msg/PowertrainStatus.msg"
    "msg/SystemStatus.msg"
    "msg/Telemetry.msg"
    "msg/TelemetryControl.msg"
    "msg/TelemetryKinematic.msg"
    "msg/TelemetryLocalization.msg"
    "msg/TelemetryMiscReport.msg"
    "msg/TelemetryPtReport.msg"
    "msg/TerrainStatus.msg"
    "msg/WatchDogHeartbeat.msg"
    "msg/WatchDogStatus.msg"
    "msg/Waypoints.msg"
    "msg/WaypointsArray.msg"
    "msg/NodeStatusShort.msg"
    "msg/NodeStatus.msg"
    "msg/MissionStatus.msg"
    "msg/LocalizationStatus.msg"
    "msg/AccelControlStatus.msg"
    "msg/VelocityPlannerStatus.msg"
)

set(srv_files
        "srv/RegisterNodeStatus.srv"
        )

rosidl_generate_interfaces(${PROJECT_NAME}
        ${msg_files}
        ${srv_files}
        DEPENDENCIES builtin_interfaces geometry_msgs nav_msgs std_msgs vision_msgs
        )


if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # uncomment the line when a copyright and license is not present in all source files
    #set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # uncomment the line when this package is not in a git repo
    #set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif ()

ament_package()
