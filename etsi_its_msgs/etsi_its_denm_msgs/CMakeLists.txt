cmake_minimum_required(VERSION 3.5)
project(etsi_its_denm_msgs)

find_package(ros_environment REQUIRED QUIET)
set(ROS_VERSION $ENV{ROS_VERSION})

# === ROS 2 (AMENT) ============================================================
if(${ROS_VERSION} EQUAL 2)

  find_package(ament_cmake REQUIRED)
  find_package(rosidl_default_generators REQUIRED)

  set(msg_files
    "msg/PositionOfPillars.msg"
    "msg/DeltaLatitude.msg"
    "msg/DangerousGoodsBasic.msg"
    "msg/WheelBaseVehicle.msg"
    "msg/StationType.msg"
    "msg/StationaryVehicleContainer.msg"
    "msg/AltitudeConfidence.msg"
    "msg/DeltaAltitude.msg"
    "msg/RelevanceDistance.msg"
    "msg/PositionOfOccupants.msg"
    "msg/LightBarSirenInUse.msg"
    "msg/RequestResponseIndication.msg"
    "msg/RelevanceTrafficDirection.msg"
    "msg/HeightLonCarr.msg"
    "msg/ReferencePosition.msg"
    "msg/DeltaLongitude.msg"
    "msg/PathHistory.msg"
    "msg/SpeedValue.msg"
    "msg/ValidityDuration.msg"
    "msg/Temperature.msg"
    "msg/SpeedConfidence.msg"
    "msg/HeadingConfidence.msg"
    "msg/VDS.msg"
    "msg/AltitudeValue.msg"
    "msg/VehicleMass.msg"
    "msg/EventPoint.msg"
    "msg/TrafficRule.msg"
    "msg/PositioningSolutionType.msg"
    "msg/ImpactReductionContainer.msg"
    "msg/LocationContainer.msg"
    "msg/SubCauseCodeType.msg"
    "msg/HeadingValue.msg"
    "msg/HardShoulderStatus.msg"
    "msg/SemiAxisLength.msg"
    "msg/SituationContainer.msg"
    "msg/PosLonCarr.msg"
    "msg/RestrictedTypes.msg"
    "msg/Speed.msg"
    "msg/InformationQuality.msg"
    "msg/AlacarteContainer.msg"
    "msg/WMInumber.msg"
    "msg/RoadWorksContainerExtended.msg"
    "msg/TimestampIts.msg"
    "msg/SpeedLimit.msg"
    "msg/DecentralizedEnvironmentalNotificationMessage.msg"
    "msg/ItineraryPath.msg"
    "msg/SequenceNumber.msg"
    "msg/ReferenceDenms.msg"
    "msg/CauseCodeType.msg"
    "msg/VehicleIdentification.msg"
    "msg/Traces.msg"
    "msg/CauseCode.msg"
    "msg/ClosedLanes.msg"
    "msg/EnergyStorageType.msg"
    "msg/Termination.msg"
    "msg/PosCentMass.msg"
    "msg/ItsPduHeader.msg"
    "msg/DangerousGoodsExtended.msg"
    "msg/Altitude.msg"
    "msg/ManagementContainer.msg"
    "msg/Heading.msg"
    "msg/TransmissionInterval.msg"
    "msg/StationID.msg"
    "msg/PathDeltaTime.msg"
    "msg/EventHistory.msg"
    "msg/PosPillar.msg"
    "msg/DeltaReferencePosition.msg"
    "msg/StationarySince.msg"
    "msg/ActionID.msg"
    "msg/PosFrontAx.msg"
    "msg/PhoneNumber.msg"
    "msg/LanePosition.msg"
    "msg/PathPoint.msg"
    "msg/NumberOfOccupants.msg"
    "msg/RoadType.msg"
    "msg/Latitude.msg"
    "msg/TurningRadius.msg"
    "msg/DrivingLaneStatus.msg"
    "msg/PosConfidenceEllipse.msg"
    "msg/Longitude.msg"
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
