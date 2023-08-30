cmake_minimum_required(VERSION 3.5)
project(etsi_its_msgs)

find_package(ros_environment REQUIRED QUIET)
set(ROS_VERSION $ENV{ROS_VERSION})
set(ROS_DISTRO $ENV{ROS_DISTRO})
add_compile_definitions(ROS_VERSION=$ENV{ROS_VERSION})
add_compile_definitions(ROS_DISTRO=$ENV{ROS_DISTRO})

# Workaround to find GeographicLib according to https://bugs.launchpad.net/ubuntu/+source/geographiclib/+bug/1805173
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};/usr/share/cmake/geographiclib")

# === ROS1 (CATKIN) ============================================================
if(${ROS_VERSION} EQUAL 1)

    find_package(catkin REQUIRED COMPONENTS
        etsi_its_cam_msgs
        geometry_msgs
    )

    find_package(GeographicLib REQUIRED)

    catkin_package(
        CFG_EXTRAS ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}-extras.cmake
        INCLUDE_DIRS include
        LIBRARIES GeographicLib
        CATKIN_DEPENDS
            etsi_its_cam_msgs
            geometry_msgs
    )

    include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${GeographicLib_INCLUDE_DIRS}
    )
    message(${GeographicLib_INCLUDE_DIRS})

    install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    )

    if(CATKIN_ENABLE_TESTING)
        catkin_add_gtest(${PROJECT_NAME}-test test/test_cam_access.cpp)
        if(TARGET ${PROJECT_NAME}-test)
            target_include_directories(${PROJECT_NAME}-test PUBLIC test)
            target_link_libraries(${PROJECT_NAME}-test ${GeographicLib_LIBRARIES} ${catkin_LIBRARIES})
        endif()
    endif()

# === ROS2 (AMENT) =============================================================
elseif(${ROS_VERSION} EQUAL 2)

    find_package(ament_cmake REQUIRED)
    find_package(etsi_its_cam_msgs REQUIRED)
    find_package(GeographicLib REQUIRED)
    find_package(geometry_msgs REQUIRED)

    add_library(etsi_its_cam_msgs_access INTERFACE)

    target_include_directories(etsi_its_cam_msgs_access INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        ${GeographicLib_INCLUDE_DIRS}
        ${etsi_its_cam_msgs_INCLUDE_DIRS}
        ${geometry_msgs_INCLUDE_DIRS}
    )

    target_link_libraries(etsi_its_cam_msgs_access INTERFACE
        ${GeographicLib_LIBRARIES}
        ${etsi_its_cam_msgs_LIBRARIES}
        ${geometry_msgs_LIBRARIES}
    )

    ament_export_targets(etsi_its_cam_msgs_accessTargets HAS_LIBRARY_TARGET)

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
            GeographicLib
            geometry_msgs
        )
    endif()

    ament_package(
        CONFIG_EXTRAS ${PROJECT_NAME}-extras.cmake
    )

endif()