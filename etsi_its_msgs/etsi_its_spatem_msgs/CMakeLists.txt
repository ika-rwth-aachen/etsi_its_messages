cmake_minimum_required(VERSION 3.5)
project(etsi_its_spatem_msgs)

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
    "msg/Altitude.msg"
    "msg/AltitudeConfidence.msg"
    "msg/AltitudeValue.msg"
    "msg/BatteryStatus.msg"
    "msg/ConnectionManeuverAssist.msg"
    "msg/ConnectionManeuverAssistaddGrpC.msg"
    "msg/ConnectionTrajectoryaddGrpC.msg"
    "msg/DeltaAltitude.msg"
    "msg/DeltaAngle.msg"
    "msg/DeltaTime.msg"
    "msg/DescriptiveName.msg"
    "msg/DSecond.msg"
    "msg/EmissionType.msg"
    "msg/EnabledLaneList.msg"
    "msg/ExceptionalCondition.msg"
    "msg/FuelType.msg"
    "msg/IntersectionID.msg"
    "msg/IntersectionReferenceID.msg"
    "msg/IntersectionState.msg"
    "msg/IntersectionStateaddGrpC.msg"
    "msg/IntersectionStateList.msg"
    "msg/IntersectionStatusObject.msg"
    "msg/ItsPduHeader.msg"
    "msg/ItsStationPosition.msg"
    "msg/ItsStationPositionList.msg"
    "msg/LaneAttributesaddGrpC.msg"
    "msg/LaneConnectionID.msg"
    "msg/LaneDataAttribute.msg"
    "msg/LaneDataAttributeList.msg"
    "msg/LaneID.msg"
    "msg/Latitude.msg"
    "msg/Longitude.msg"
    "msg/ManeuverAssistList.msg"
    "msg/MapDataaddGrpC.msg"
    "msg/MergeDivergeNodeAngle.msg"
    "msg/MinuteOfTheYear.msg"
    "msg/MovementEvent.msg"
    "msg/MovementEventaddGrpC.msg"
    "msg/MovementEventList.msg"
    "msg/MovementList.msg"
    "msg/MovementPhaseState.msg"
    "msg/MovementState.msg"
    "msg/MsgCount.msg"
    "msg/Node.msg"
    "msg/NodeAttributeSetaddGrpC.msg"
    "msg/NodeAttributeSetXY.msg"
    "msg/NodeAttributeXY.msg"
    "msg/NodeAttributeXYList.msg"
    "msg/NodeLink.msg"
    "msg/NodeLLmD64b.msg"
    "msg/NodeOffsetPointXY.msg"
    "msg/NodeSetXY.msg"
    "msg/NodeXY.msg"
    "msg/NodeXY20b.msg"
    "msg/NodeXY22b.msg"
    "msg/NodeXY24b.msg"
    "msg/NodeXY26b.msg"
    "msg/NodeXY28b.msg"
    "msg/NodeXY32b.msg"
    "msg/OffsetB10.msg"
    "msg/OffsetB11.msg"
    "msg/OffsetB12.msg"
    "msg/OffsetB13.msg"
    "msg/OffsetB14.msg"
    "msg/OffsetB16.msg"
    "msg/PedestrianBicycleDetect.msg"
    "msg/Position3DaddGrpC.msg"
    "msg/PrioritizationResponse.msg"
    "msg/PrioritizationResponseList.msg"
    "msg/PrioritizationResponseStatus.msg"
    "msg/PtvRequestType.msg"
    "msg/RegionalExtension.msg"
    "msg/RegionId.msg"
    "msg/RegulatorySpeedLimit.msg"
    "msg/RejectedReason.msg"
    "msg/RequestorDescriptionaddGrpC.msg"
    "msg/RestrictionClassID.msg"
    "msg/RestrictionUserTypeaddGrpC.msg"
    "msg/RoadRegulatorID.msg"
    "msg/RoadwayCrownAngle.msg"
    "msg/SegmentAttributeXY.msg"
    "msg/SegmentAttributeXYList.msg"
    "msg/SignalGroupID.msg"
    "msg/SignalHeadLocation.msg"
    "msg/SignalHeadLocationList.msg"
    "msg/SignalStatusPackageaddGrpC.msg"
    "msg/SPAT.msg"
    "msg/SPATEM.msg"
    "msg/SpeedAdvice.msg"
    "msg/SpeedConfidenceDSRC.msg"
    "msg/SpeedLimitList.msg"
    "msg/SpeedLimitType.msg"
    "msg/StationID.msg"
    "msg/TimeChangeDetails.msg"
    "msg/TimeIntervalConfidence.msg"
    "msg/TimeMark.msg"
    "msg/TimeReference.msg"
    "msg/VehicleHeight.msg"
    "msg/VehicleMass.msg"
    "msg/Velocity.msg"
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
