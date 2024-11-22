cmake_minimum_required(VERSION 3.5)
project(etsi_its_mapem_ts_msgs)

find_package(ros_environment REQUIRED QUIET)
set(ROS_VERSION $ENV{ROS_VERSION})

# === ROS 2 (AMENT) ============================================================
if(${ROS_VERSION} EQUAL 2)

  find_package(ament_cmake REQUIRED)
  find_package(rosidl_default_generators REQUIRED)

  set(msg_files
    "msg/AllowedManeuvers.msg"
    "msg/Angle.msg"
    "msg/ApproachID.msg"
    "msg/ComputedLane.msg"
    "msg/ConnectingLane.msg"
    "msg/Connection.msg"
    "msg/ConnectsToList.msg"
    "msg/DataParameters.msg"
    "msg/DeltaAngle.msg"
    "msg/DescriptiveName.msg"
    "msg/DrivenLineOffsetLg.msg"
    "msg/DrivenLineOffsetSm.msg"
    "msg/Elevation.msg"
    "msg/GenericLane.msg"
    "msg/IntersectionGeometry.msg"
    "msg/IntersectionGeometryList.msg"
    "msg/IntersectionID.msg"
    "msg/IntersectionReferenceID.msg"
    "msg/ItsPduHeader.msg"
    "msg/LaneAttributes.msg"
    "msg/LaneAttributesBarrier.msg"
    "msg/LaneAttributesBike.msg"
    "msg/LaneAttributesCrosswalk.msg"
    "msg/LaneAttributesParking.msg"
    "msg/LaneAttributesSidewalk.msg"
    "msg/LaneAttributesStriping.msg"
    "msg/LaneAttributesTrackedVehicle.msg"
    "msg/LaneAttributesVehicle.msg"
    "msg/LaneConnectionID.msg"
    "msg/LaneDataAttribute.msg"
    "msg/LaneDataAttributeList.msg"
    "msg/LaneDirection.msg"
    "msg/LaneID.msg"
    "msg/LaneList.msg"
    "msg/LaneSharing.msg"
    "msg/LaneTypeAttributes.msg"
    "msg/LaneWidth.msg"
    "msg/Latitude.msg"
    "msg/LayerID.msg"
    "msg/LayerType.msg"
    "msg/Longitude.msg"
    "msg/MAPEM.msg"
    "msg/MapData.msg"
    "msg/MergeDivergeNodeAngle.msg"
    "msg/MinuteOfTheYear.msg"
    "msg/MsgCount.msg"
    "msg/NodeAttributeSetXY.msg"
    "msg/NodeAttributeXY.msg"
    "msg/NodeAttributeXYList.msg"
    "msg/NodeLLmD64b.msg"
    "msg/NodeListXY.msg"
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
    "msg/OverlayLaneList.msg"
    "msg/Position3D.msg"
    "msg/PreemptPriorityList.msg"
    "msg/RegulatorySpeedLimit.msg"
    "msg/RestrictionAppliesTo.msg"
    "msg/RestrictionClassAssignment.msg"
    "msg/RestrictionClassID.msg"
    "msg/RestrictionClassList.msg"
    "msg/RestrictionUserType.msg"
    "msg/RestrictionUserTypeList.msg"
    "msg/RoadLaneSetList.msg"
    "msg/RoadRegulatorID.msg"
    "msg/RoadSegment.msg"
    "msg/RoadSegmentID.msg"
    "msg/RoadSegmentList.msg"
    "msg/RoadSegmentReferenceID.msg"
    "msg/RoadwayCrownAngle.msg"
    "msg/ScaleB12.msg"
    "msg/SegmentAttributeXY.msg"
    "msg/SegmentAttributeXYList.msg"
    "msg/SignalControlZone.msg"
    "msg/SignalGroupID.msg"
    "msg/SpeedLimitList.msg"
    "msg/SpeedLimitType.msg"
    "msg/StationID.msg"
    "msg/Velocity.msg"
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
