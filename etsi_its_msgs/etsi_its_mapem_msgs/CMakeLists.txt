cmake_minimum_required(VERSION 3.5)
project(etsi_its_mapem_msgs)

find_package(ros_environment REQUIRED QUIET)
set(ROS_VERSION $ENV{ROS_VERSION})

# === ROS 2 (AMENT) ============================================================
if(${ROS_VERSION} EQUAL 2)

  find_package(ament_cmake REQUIRED)
  find_package(rosidl_default_generators REQUIRED)

  set(msg_files
    # msg/AddGrpC
    "BatteryStatus.msg"
    "ConnectionManeuverAssistaddGrpC.msg"
    "ConnectionTrajectoryaddGrpC.msg"
    "EmissionType.msg"
    "ExceptionalCondition.msg"
    "IntersectionStateaddGrpC.msg"
    "ItsStationPositionList.msg"
    "ItsStationPosition.msg"
    "LaneAttributesaddGrpC.msg"
    "MapDataaddGrpC.msg"
    "MovementEventaddGrpC.msg"
    "NodeAttributeSetaddGrpC.msg"
    "NodeLink.msg"
    "Node.msg"
    "Position3DaddGrpC.msg"
    "PrioritizationResponseList.msg"
    "PrioritizationResponse.msg"
    "PtvRequestType.msg"
    "RejectedReason.msg"
    "RequestorDescriptionaddGrpC.msg"
    "RestrictionUserTypeaddGrpC.msg"
    "SignalHeadLocationList.msg"
    "SignalHeadLocation.msg"
    "SignalStatusPackageaddGrpC.msg"
    "TimeReference.msg"
    # msg/DSRC
    "AllowedManeuvers.msg"
    "Angle.msg"
    "ApproachID.msg"
    "ComputedLane.msg"
    "ConnectingLane.msg"
    "Connection.msg"
    "ConnectsToList.msg"
    "DataParameters.msg"
    "DeltaAngle.msg"
    "DeltaTime.msg"
    "DescriptiveName.msg"
    "DrivenLineOffsetLg.msg"
    "DrivenLineOffsetSm.msg"
    "Elevation.msg"
    "FuelType.msg"
    "GenericLane.msg"
    "IntersectionGeometryList.msg"
    "IntersectionGeometry.msg"
    "IntersectionID.msg"
    "IntersectionReferenceID.msg"
    "LaneAttributesBarrier.msg"
    "LaneAttributesBike.msg"
    "LaneAttributesCrosswalk.msg"
    "LaneAttributes.msg"
    "LaneAttributesParking.msg"
    "LaneAttributesSidewalk.msg"
    "LaneAttributesStriping.msg"
    "LaneAttributesTrackedVehicle.msg"
    "LaneAttributesVehicle.msg"
    "LaneConnectionID.msg"
    "LaneDataAttributeList.msg"
    "LaneDataAttribute.msg"
    "LaneDirection.msg"
    "LaneID.msg"
    "LaneList.msg"
    "LaneSharing.msg"
    "LaneTypeAttributes.msg"
    "LaneWidth.msg"
    "LayerID.msg"
    "LayerType.msg"
    "MapData.msg"
    "MergeDivergeNodeAngle.msg"
    "MinuteOfTheYear.msg"
    "MsgCount.msg"
    "NodeAttributeSetXY.msg"
    "NodeAttributeXYList.msg"
    "NodeAttributeXY.msg"
    "NodeListXY.msg"
    "NodeLLmD64b.msg"
    "NodeOffsetPointXY.msg"
    "NodeSetXY.msg"
    "NodeXY20b.msg"
    "NodeXY22b.msg"
    "NodeXY24b.msg"
    "NodeXY26b.msg"
    "NodeXY28b.msg"
    "NodeXY32b.msg"
    "NodeXY.msg"
    "OffsetB10.msg"
    "OffsetB11.msg"
    "OffsetB12.msg"
    "OffsetB13.msg"
    "OffsetB14.msg"
    "OffsetB16.msg"
    "OverlayLaneList.msg"
    "Position3D.msg"
    "PreemptPriorityList.msg"
    "PrioritizationResponseStatus.msg"
    "RegionalExtension.msg"
    "RegionId.msg"
    "RegulatorySpeedLimit.msg"
    "RestrictionAppliesTo.msg"
    "RestrictionClassAssignment.msg"
    "RestrictionClassID.msg"
    "RestrictionClassList.msg"
    "RestrictionUserTypeList.msg"
    "RestrictionUserType.msg"
    "RoadLaneSetList.msg"
    "RoadRegulatorID.msg"
    "RoadSegmentID.msg"
    "RoadSegmentList.msg"
    "RoadSegment.msg"
    "RoadSegmentReferenceID.msg"
    "RoadwayCrownAngle.msg"
    "ScaleB12.msg"
    "SegmentAttributeXYList.msg"
    "SegmentAttributeXY.msg"
    "SignalControlZone.msg"
    "SignalGroupID.msg"
    "SpeedLimitList.msg"
    "SpeedLimitType.msg"
    "VehicleHeight.msg"
    "Velocity.msg"
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
    # msg/MAPEM-PDU-Descriptions
    "msg/MAPEM-PDU-Descriptions/MAPEM.msg"
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

  add_message_files(DIRECTORY msg/DENM-PDU-Descriptions)
  add_message_files(DIRECTORY msg/ITS-Container)

  generate_messages(
    DEPENDENCIES std_msgs
  )

  catkin_package(
    CATKIN_DEPENDS
      message_runtime
      std_msgs
  )

endif()
