cmake_minimum_required(VERSION 3.5)
project(etsi_its_msgs)

find_package(ros_environment REQUIRED QUIET)
set(ROS_VERSION $ENV{ROS_VERSION})

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
    find_package(etsi_its_cam_msgs REQUIRED)

    add_library(etsi_its_cam_msgs_access INTERFACE)

    target_include_directories(etsi_its_cam_msgs_access INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    )

    ament_export_targets(etsi_its_cam_msgs_accessTargets HAS_LIBRARY_TARGET)
    ament_export_dependencies(etsi_its_cam_msgs)

    install(
        DIRECTORY include/
        DESTINATION include
    )

    install(TARGETS etsi_its_cam_msgs_access
        EXPORT etsi_its_cam_msgs_accessTargets
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib
        INCLUDES DESTINATION include
    )

    if(BUILD_TESTING)
        find_package(ament_cmake_gtest REQUIRED)
        ament_add_gtest(etsi_its_cam_msgs_access-test test/test_cam_access.ros2.cpp)
        target_include_directories(etsi_its_cam_msgs_access-test PUBLIC
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:include>
        )
        target_include_directories(etsi_its_cam_msgs_access-test PUBLIC test)
        ament_target_dependencies(etsi_its_cam_msgs_access-test
            etsi_its_cam_msgs
        )
    endif()

    ament_package()

endif()