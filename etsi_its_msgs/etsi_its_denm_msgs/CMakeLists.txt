cmake_minimum_required(VERSION 3.5)
project(etsi_its_denm_msgs)

find_package(ros_environment REQUIRED QUIET)
set(ROS_VERSION $ENV{ROS_VERSION})

# === ROS 2 (AMENT) ============================================================
if(${ROS_VERSION} EQUAL 2)

  find_package(ament_cmake REQUIRED)
  find_package(rosidl_default_generators REQUIRED)

  set(msg_files
    # msg/DENM-PDU-Descriptions
    "msg/DENM-PDU-Descriptions/AlacarteContainer.msg"
    "msg/DENM-PDU-Descriptions/DecentralizedEnvironmentalNotificationMessage.msg"
    "msg/DENM-PDU-Descriptions/DENM.msg"
    "msg/DENM-PDU-Descriptions/ImpactReductionContainer.msg"
    "msg/DENM-PDU-Descriptions/LocationContainer.msg"
    "msg/DENM-PDU-Descriptions/ManagementContainer.msg"
    "msg/DENM-PDU-Descriptions/ReferenceDenms.msg"
    "msg/DENM-PDU-Descriptions/RoadWorksContainerExtended.msg"
    "msg/DENM-PDU-Descriptions/SituationContainer.msg"
    "msg/DENM-PDU-Descriptions/StationaryVehicleContainer.msg"
    "msg/DENM-PDU-Descriptions/Termination.msg"
    # msg/ITS-Container
    "msg/ITS-Container/ActionID.msg"
    "msg/ITS-Container/AltitudeConfidence.msg"
    "msg/ITS-Container/Altitude.msg"
    "msg/ITS-Container/AltitudeValue.msg"
    "msg/ITS-Container/CauseCode.msg"
    "msg/ITS-Container/CauseCodeType.msg"
    "msg/ITS-Container/ClosedLanes.msg"
    "msg/ITS-Container/DangerousGoodsBasic.msg"
    "msg/ITS-Container/DangerousGoodsExtended.msg"
    "msg/ITS-Container/DeltaAltitude.msg"
    "msg/ITS-Container/DeltaLatitude.msg"
    "msg/ITS-Container/DeltaLongitude.msg"
    "msg/ITS-Container/DeltaReferencePosition.msg"
    "msg/ITS-Container/DrivingLaneStatus.msg"
    "msg/ITS-Container/EnergyStorageType.msg"
    "msg/ITS-Container/EventHistory.msg"
    "msg/ITS-Container/EventPoint.msg"
    "msg/ITS-Container/HardShoulderStatus.msg"
    "msg/ITS-Container/HeadingConfidence.msg"
    "msg/ITS-Container/Heading.msg"
    "msg/ITS-Container/HeadingValue.msg"
    "msg/ITS-Container/HeightLonCarr.msg"
    "msg/ITS-Container/InformationQuality.msg"
    "msg/ITS-Container/ItineraryPath.msg"
    "msg/ITS-Container/ItsPduHeader.msg"
    "msg/ITS-Container/LanePosition.msg"
    "msg/ITS-Container/Latitude.msg"
    "msg/ITS-Container/LightBarSirenInUse.msg"
    "msg/ITS-Container/Longitude.msg"
    "msg/ITS-Container/NumberOfOccupants.msg"
    "msg/ITS-Container/PathDeltaTime.msg"
    "msg/ITS-Container/PathHistory.msg"
    "msg/ITS-Container/PathPoint.msg"
    "msg/ITS-Container/PhoneNumber.msg"
    "msg/ITS-Container/PosCentMass.msg"
    "msg/ITS-Container/PosConfidenceEllipse.msg"
    "msg/ITS-Container/PosFrontAx.msg"
    "msg/ITS-Container/PositioningSolutionType.msg"
    "msg/ITS-Container/PositionOfOccupants.msg"
    "msg/ITS-Container/PositionOfPillars.msg"
    "msg/ITS-Container/PosLonCarr.msg"
    "msg/ITS-Container/PosPillar.msg"
    "msg/ITS-Container/ReferencePosition.msg"
    "msg/ITS-Container/RelevanceDistance.msg"
    "msg/ITS-Container/RelevanceTrafficDirection.msg"
    "msg/ITS-Container/RequestResponseIndication.msg"
    "msg/ITS-Container/RestrictedTypes.msg"
    "msg/ITS-Container/RoadType.msg"
    "msg/ITS-Container/SemiAxisLength.msg"
    "msg/ITS-Container/SequenceNumber.msg"
    "msg/ITS-Container/SpeedConfidence.msg"
    "msg/ITS-Container/SpeedLimit.msg"
    "msg/ITS-Container/Speed.msg"
    "msg/ITS-Container/SpeedValue.msg"
    "msg/ITS-Container/StationarySince.msg"
    "msg/ITS-Container/StationID.msg"
    "msg/ITS-Container/StationType.msg"
    "msg/ITS-Container/SubCauseCodeType.msg"
    "msg/ITS-Container/Temperature.msg"
    "msg/ITS-Container/TimestampIts.msg"
    "msg/ITS-Container/Traces.msg"
    "msg/ITS-Container/TrafficRule.msg"
    "msg/ITS-Container/TransmissionInterval.msg"
    "msg/ITS-Container/TurningRadius.msg"
    "msg/ITS-Container/ValidityDuration.msg"
    "msg/ITS-Container/VDS.msg"
    "msg/ITS-Container/VehicleIdentification.msg"
    "msg/ITS-Container/VehicleMass.msg"
    "msg/ITS-Container/WheelBaseVehicle.msg"
    "msg/ITS-Container/WMInumber.msg"
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
