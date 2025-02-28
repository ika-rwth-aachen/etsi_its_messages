cmake_minimum_required(VERSION 3.5)
project(etsi_its_mcm_uulm_msgs)

find_package(ros_environment REQUIRED QUIET)
set(ROS_VERSION $ENV{ROS_VERSION})

# === ROS 2 (AMENT) ============================================================
if(${ROS_VERSION} EQUAL 2)

  find_package(ament_cmake REQUIRED)
  find_package(rosidl_default_generators REQUIRED)

  set(msg_files
    "msg/AdviceResponse.msg"
    "msg/AdviceUpdateID.msg"
    "msg/Altitude.msg"
    "msg/AltitudeConfidence.msg"
    "msg/AltitudeValue.msg"
    "msg/BasicContainer.msg"
    "msg/CartesianCoordinateLarge.msg"
    "msg/ConfirmationRequiredFlag.msg"
    "msg/DesiredRoute.msg"
    "msg/GenerationDeltaTime.msg"
    "msg/HeadingValue.msg"
    "msg/ItsPduHeader.msg"
    "msg/Latitude.msg"
    "msg/Longitude.msg"
    "msg/LongitudinalManeuverWaypointContainer.msg"
    "msg/LongitudinalWaypoint.msg"
    "msg/MCM.msg"
    "msg/ManeuverConstraints.msg"
    "msg/ManeuverContainer.msg"
    "msg/ManeuverCoordinationMessage.msg"
    "msg/ManeuverID.msg"
    "msg/ManeuverParameters.msg"
    "msg/ManeuverResponse.msg"
    "msg/ManeuverType.msg"
    "msg/McmParameters.msg"
    "msg/MessageId.msg"
    "msg/OrdinalNumber1B.msg"
    "msg/ParticipatingRoadUserIDContainer.msg"
    "msg/ParticipatingRoadUserIndex.msg"
    "msg/PlannedTrajectory.msg"
    "msg/Polygon.msg"
    "msg/PositionConfidenceEllipse.msg"
    "msg/ReferencePositionWithConfidence.msg"
    "msg/RoadUserContainer.msg"
    "msg/RoadUserLength.msg"
    "msg/RoadUserState.msg"
    "msg/RoadUserType.msg"
    "msg/RoadUserWidth.msg"
    "msg/SemiAxisLength.msg"
    "msg/SpeedValue.msg"
    "msg/StationId.msg"
    "msg/SuggestedManeuver.msg"
    "msg/SuggestedManeuverContainer.msg"
    "msg/TerminationStatus.msg"
    "msg/TrafficParticipantType.msg"
    "msg/TrajectoryPoint.msg"
    "msg/TrajectoryPointContainer.msg"
    "msg/TrajectoryPointDeltaTime.msg"
    "msg/TrajectoryStartDeltaTime.msg"
    "msg/Waypoint.msg"
    "msg/WaypointDeltaTime.msg"
    "msg/Wgs84AngleValue.msg"
    "msg/YieldToRoadUserContainer.msg"
  )

  rosidl_generate_interfaces(${PROJECT_NAME}
    ${msg_files}
  )

  ament_export_dependencies(rosidl_default_runtime)

  ament_package()

# === ROS (CATKIN) =============================================================
elseif(${ROS_VERSION} EQUAL 1)

  find_package(catkin REQUIRED COMPONENTS
    message_generation
    std_msgs
  )

  add_message_files(DIRECTORY msg)

  generate_messages(
    DEPENDENCIES std_msgs
  )

  catkin_package(
    CATKIN_DEPENDS
      message_runtime
      std_msgs
  )

endif()
