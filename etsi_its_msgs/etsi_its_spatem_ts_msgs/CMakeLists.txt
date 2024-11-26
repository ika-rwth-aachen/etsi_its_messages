cmake_minimum_required(VERSION 3.5)
project(etsi_its_spatem_ts_msgs)

find_package(ros_environment REQUIRED QUIET)
set(ROS_VERSION $ENV{ROS_VERSION})

# === ROS 2 (AMENT) ============================================================
if(${ROS_VERSION} EQUAL 2)

  find_package(ament_cmake REQUIRED)
  find_package(rosidl_default_generators REQUIRED)

  set(msg_files
    "msg/AdvisorySpeed.msg"
    "msg/AdvisorySpeedList.msg"
    "msg/AdvisorySpeedType.msg"
    "msg/ConnectionManeuverAssist.msg"
    "msg/DSecond.msg"
    "msg/DescriptiveName.msg"
    "msg/EnabledLaneList.msg"
    "msg/IntersectionID.msg"
    "msg/IntersectionReferenceID.msg"
    "msg/IntersectionState.msg"
    "msg/IntersectionStateList.msg"
    "msg/IntersectionStatusObject.msg"
    "msg/ItsPduHeader.msg"
    "msg/LaneConnectionID.msg"
    "msg/LaneID.msg"
    "msg/ManeuverAssistList.msg"
    "msg/MinuteOfTheYear.msg"
    "msg/MovementEvent.msg"
    "msg/MovementEventList.msg"
    "msg/MovementList.msg"
    "msg/MovementPhaseState.msg"
    "msg/MovementState.msg"
    "msg/MsgCount.msg"
    "msg/PedestrianBicycleDetect.msg"
    "msg/RestrictionClassID.msg"
    "msg/RoadRegulatorID.msg"
    "msg/SPAT.msg"
    "msg/SPATEM.msg"
    "msg/SignalGroupID.msg"
    "msg/SpeedAdvice.msg"
    "msg/SpeedConfidenceDSRC.msg"
    "msg/StationID.msg"
    "msg/TimeChangeDetails.msg"
    "msg/TimeIntervalConfidence.msg"
    "msg/TimeMark.msg"
    "msg/WaitOnStopline.msg"
    "msg/ZoneLength.msg"
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
