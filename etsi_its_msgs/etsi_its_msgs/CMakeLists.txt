cmake_minimum_required(VERSION 2.8.3)
project(etsi_its_msgs)

find_package(ros_environment REQUIRED QUIET)
set(ROS_VERSION $ENV{ROS_VERSION})
set(ROS_DISTRO $ENV{ROS_DISTRO})
add_compile_definitions(ROS_VERSION=$ENV{ROS_VERSION})
add_compile_definitions(ROS_DISTRO=$ENV{ROS_DISTRO})

# === ROS1 (CATKIN) ============================================================
if(${ROS_VERSION} EQUAL 1)

    find_package(catkin REQUIRED COMPONENTS
        etsi_its_cam_msgs
    )

    catkin_package(
        INCLUDE_DIRS include
        CATKIN_DEPENDS
            etsi_its_cam_msgs
    )

    include_directories(
        include
        ${catkin_INCLUDE_DIRS}
    )

    install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    )

    if(CATKIN_ENABLE_TESTING)
        catkin_add_gtest(${PROJECT_NAME}-test test/test_cam_access.cpp)
        if(TARGET ${PROJECT_NAME}-test)
            target_include_directories(${PROJECT_NAME}-test PUBLIC test)
        endif()
    endif()

# === ROS2 (AMENT) =============================================================
elseif(${ROS_VERSION} EQUAL 2)

    find_package(ament_cmake REQUIRED)

endif()