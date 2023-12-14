cmake_minimum_required(VERSION 3.5)
project(etsi_its_spatem_msgs)

find_package(ros_environment REQUIRED QUIET)
set(ROS_VERSION $ENV{ROS_VERSION})

# === ROS 2 (AMENT) ============================================================
if(${ROS_VERSION} EQUAL 2)

  find_package(ament_cmake REQUIRED)
  find_package(rosidl_default_generators REQUIRED)

  set(msg_files
    # msg/AddGrpC
    "msg/AddGrpC/BatteryStatus.msg"
    "msg/AddGrpC/ConnectionManeuverAssistaddGrpC.msg"
    "msg/AddGrpC/ConnectionTrajectoryaddGrpC.msg"
    "msg/AddGrpC/EmissionType.msg"
    "msg/AddGrpC/ExceptionalCondition.msg"
    "msg/AddGrpC/IntersectionStateaddGrpC.msg"
    "msg/AddGrpC/ItsStationPosition.msg"
    "msg/AddGrpC/ItsStationPositionList.msg"
    "msg/AddGrpC/LaneAttributesaddGrpC.msg"
    "msg/AddGrpC/MapDataaddGrpC.msg"
    "msg/AddGrpC/MovementEventaddGrpC.msg"
    "msg/AddGrpC/Node.msg"
    "msg/AddGrpC/NodeAttributeSetaddGrpC.msg"
    "msg/AddGrpC/NodeLink.msg"
    "msg/AddGrpC/Position3DaddGrpC.msg"
    "msg/AddGrpC/PrioritizationResponse.msg"
    "msg/AddGrpC/PrioritizationResponseList.msg"
    "msg/AddGrpC/PtvRequestType.msg"
    "msg/AddGrpC/RejectedReason.msg"
    "msg/AddGrpC/RequestorDescriptionaddGrpC.msg"
    "msg/AddGrpC/RestrictionUserTypeaddGrpC.msg"
    "msg/AddGrpC/SignalHeadLocation.msg"
    "msg/AddGrpC/SignalHeadLocationList.msg"
    "msg/AddGrpC/SignalStatusPackageaddGrpC.msg"
    "msg/AddGrpC/TimeReference.msg"
    # msg/DSRC
    "msg/DSRC/AdvisorySpeed.msg"
    "msg/DSRC/AdvisorySpeedList.msg"
    "msg/DSRC/AdvisorySpeedType.msg"
    "msg/DSRC/ConnectionManeuverAssist.msg"
    "msg/DSRC/DSecond.msg"
    "msg/DSRC/DeltaAngle.msg"
    "msg/DSRC/DeltaTime.msg"
    "msg/DSRC/DescriptiveName.msg"
    "msg/DSRC/EnabledLaneList.msg"
    "msg/DSRC/FuelType.msg"
    "msg/DSRC/IntersectionID.msg"
    "msg/DSRC/IntersectionReferenceID.msg"
    "msg/DSRC/IntersectionState.msg"
    "msg/DSRC/IntersectionStateList.msg"
    "msg/DSRC/IntersectionStatusObject.msg"
    "msg/DSRC/LaneConnectionID.msg"
    "msg/DSRC/LaneDataAttribute.msg"
    "msg/DSRC/LaneDataAttributeList.msg"
    "msg/DSRC/LaneID.msg"
    "msg/DSRC/ManeuverAssistList.msg"
    "msg/DSRC/MergeDivergeNodeAngle.msg"
    "msg/DSRC/MinuteOfTheYear.msg"
    "msg/DSRC/MovementEvent.msg"
    "msg/DSRC/MovementEventList.msg"
    "msg/DSRC/MovementList.msg"
    "msg/DSRC/MovementPhaseState.msg"
    "msg/DSRC/MovementState.msg"
    "msg/DSRC/MsgCount.msg"
    "msg/DSRC/NodeAttributeSetXY.msg"
    "msg/DSRC/NodeAttributeXY.msg"
    "msg/DSRC/NodeAttributeXYList.msg"
    "msg/DSRC/NodeLLmD64b.msg"
    "msg/DSRC/NodeOffsetPointXY.msg"
    "msg/DSRC/NodeSetXY.msg"
    "msg/DSRC/NodeXY.msg"
    "msg/DSRC/NodeXY20b.msg"
    "msg/DSRC/NodeXY22b.msg"
    "msg/DSRC/NodeXY24b.msg"
    "msg/DSRC/NodeXY26b.msg"
    "msg/DSRC/NodeXY28b.msg"
    "msg/DSRC/NodeXY32b.msg"
    "msg/DSRC/OffsetB10.msg"
    "msg/DSRC/OffsetB11.msg"
    "msg/DSRC/OffsetB12.msg"
    "msg/DSRC/OffsetB13.msg"
    "msg/DSRC/OffsetB14.msg"
    "msg/DSRC/OffsetB16.msg"
    "msg/DSRC/PedestrianBicycleDetect.msg"
    "msg/DSRC/PrioritizationResponseStatus.msg"
    "msg/DSRC/RegionId.msg"
    "msg/DSRC/RegionalExtension.msg"
    "msg/DSRC/RegulatorySpeedLimit.msg"
    "msg/DSRC/RestrictionClassID.msg"
    "msg/DSRC/RoadRegulatorID.msg"
    "msg/DSRC/RoadwayCrownAngle.msg"
    "msg/DSRC/SPAT.msg"
    "msg/DSRC/SegmentAttributeXY.msg"
    "msg/DSRC/SegmentAttributeXYList.msg"
    "msg/DSRC/SignalGroupID.msg"
    "msg/DSRC/SpeedAdvice.msg"
    "msg/DSRC/SpeedConfidenceDSRC.msg"
    "msg/DSRC/SpeedLimitList.msg"
    "msg/DSRC/SpeedLimitType.msg"
    "msg/DSRC/TimeChangeDetails.msg"
    "msg/DSRC/TimeIntervalConfidence.msg"
    "msg/DSRC/TimeMark.msg"
    "msg/DSRC/VehicleHeight.msg"
    "msg/DSRC/Velocity.msg"
    "msg/DSRC/WaitOnStopline.msg"
    "msg/DSRC/ZoneLength.msg"
    # msg/ITS-Container
    "msg/ITS-Container/Altitude.msg"
    "msg/ITS-Container/AltitudeConfidence.msg"
    "msg/ITS-Container/AltitudeValue.msg"
    "msg/ITS-Container/DeltaAltitude.msg"
    "msg/ITS-Container/ItsPduHeader.msg"
    "msg/ITS-Container/Latitude.msg"
    "msg/ITS-Container/Longitude.msg"
    "msg/ITS-Container/StationID.msg"
    "msg/ITS-Container/VehicleMass.msg"
    # msg/SPATEM-PDU-Descriptions
    "msg/SPATEM-PDU-Descriptions/SPATEM.msg"
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

  add_message_files(DIRECTORY msg/AddGrpC)
  add_message_files(DIRECTORY msg/DSRC)
  add_message_files(DIRECTORY msg/ITS-Container)
  add_message_files(DIRECTORY msg/SPATEM-PDU-Descriptions)

  generate_messages(
    DEPENDENCIES std_msgs
  )

  catkin_package(
    CATKIN_DEPENDS
      message_runtime
      std_msgs
  )

endif()
