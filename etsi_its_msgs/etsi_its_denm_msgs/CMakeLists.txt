cmake_minimum_required(VERSION 3.5)
project(etsi_its_denm_msgs)

find_package(ros_environment REQUIRED QUIET)
set(ROS_VERSION $ENV{ROS_VERSION})

# === ROS 2 (AMENT) ============================================================
if(${ROS_VERSION} EQUAL 2)

  find_package(ament_cmake REQUIRED)
  find_package(rosidl_default_generators REQUIRED)

  set(msg_files
    "msg/AccidentSubCauseCode.msg"
    "msg/ActionID.msg"
    "msg/AdverseWeatherConditionAdhesionSubCauseCode.msg"
    "msg/AdverseWeatherConditionExtremeWeatherConditionSubCauseCode.msg"
    "msg/AdverseWeatherConditionPrecipitationSubCauseCode.msg"
    "msg/AdverseWeatherConditionVisibilitySubCauseCode.msg"
    "msg/AlacarteContainer.msg"
    "msg/Altitude.msg"
    "msg/AltitudeConfidence.msg"
    "msg/AltitudeValue.msg"
    "msg/CauseCode.msg"
    "msg/CauseCodeType.msg"
    "msg/ClosedLanes.msg"
    "msg/CollisionRiskSubCauseCode.msg"
    "msg/DENM.msg"
    "msg/DangerousEndOfQueueSubCauseCode.msg"
    "msg/DangerousGoodsBasic.msg"
    "msg/DangerousGoodsExtended.msg"
    "msg/DangerousSituationSubCauseCode.msg"
    "msg/DecentralizedEnvironmentalNotificationMessage.msg"
    "msg/DeltaAltitude.msg"
    "msg/DeltaLatitude.msg"
    "msg/DeltaLongitude.msg"
    "msg/DeltaReferencePosition.msg"
    "msg/DrivingLaneStatus.msg"
    "msg/EmergencyVehicleApproachingSubCauseCode.msg"
    "msg/EnergyStorageType.msg"
    "msg/EventHistory.msg"
    "msg/EventPoint.msg"
    "msg/HardShoulderStatus.msg"
    "msg/HazardousLocationAnimalOnTheRoadSubCauseCode.msg"
    "msg/HazardousLocationDangerousCurveSubCauseCode.msg"
    "msg/HazardousLocationObstacleOnTheRoadSubCauseCode.msg"
    "msg/HazardousLocationSurfaceConditionSubCauseCode.msg"
    "msg/Heading.msg"
    "msg/HeadingConfidence.msg"
    "msg/HeadingValue.msg"
    "msg/HeightLonCarr.msg"
    "msg/HumanPresenceOnTheRoadSubCauseCode.msg"
    "msg/HumanProblemSubCauseCode.msg"
    "msg/ImpactReductionContainer.msg"
    "msg/InformationQuality.msg"
    "msg/ItineraryPath.msg"
    "msg/ItsPduHeader.msg"
    "msg/LanePosition.msg"
    "msg/Latitude.msg"
    "msg/LightBarSirenInUse.msg"
    "msg/LocationContainer.msg"
    "msg/Longitude.msg"
    "msg/ManagementContainer.msg"
    "msg/NumberOfOccupants.msg"
    "msg/PathDeltaTime.msg"
    "msg/PathHistory.msg"
    "msg/PathPoint.msg"
    "msg/PhoneNumber.msg"
    "msg/PosCentMass.msg"
    "msg/PosConfidenceEllipse.msg"
    "msg/PosFrontAx.msg"
    "msg/PosLonCarr.msg"
    "msg/PosPillar.msg"
    "msg/PositionOfOccupants.msg"
    "msg/PositionOfPillars.msg"
    "msg/PositioningSolutionType.msg"
    "msg/PostCrashSubCauseCode.msg"
    "msg/ReferenceDenms.msg"
    "msg/ReferencePosition.msg"
    "msg/RelevanceDistance.msg"
    "msg/RelevanceTrafficDirection.msg"
    "msg/RequestResponseIndication.msg"
    "msg/RescueAndRecoveryWorkInProgressSubCauseCode.msg"
    "msg/RestrictedTypes.msg"
    "msg/RoadType.msg"
    "msg/RoadWorksContainerExtended.msg"
    "msg/RoadworksSubCauseCode.msg"
    "msg/SemiAxisLength.msg"
    "msg/SequenceNumber.msg"
    "msg/SignalViolationSubCauseCode.msg"
    "msg/SituationContainer.msg"
    "msg/SlowVehicleSubCauseCode.msg"
    "msg/Speed.msg"
    "msg/SpeedConfidence.msg"
    "msg/SpeedLimit.msg"
    "msg/SpeedValue.msg"
    "msg/StationID.msg"
    "msg/StationType.msg"
    "msg/StationarySince.msg"
    "msg/StationaryVehicleContainer.msg"
    "msg/StationaryVehicleSubCauseCode.msg"
    "msg/SubCauseCodeType.msg"
    "msg/Temperature.msg"
    "msg/Termination.msg"
    "msg/TimestampIts.msg"
    "msg/Traces.msg"
    "msg/TrafficConditionSubCauseCode.msg"
    "msg/TrafficRule.msg"
    "msg/TransmissionInterval.msg"
    "msg/TurningRadius.msg"
    "msg/VDS.msg"
    "msg/ValidityDuration.msg"
    "msg/VehicleBreakdownSubCauseCode.msg"
    "msg/VehicleIdentification.msg"
    "msg/VehicleMass.msg"
    "msg/WMInumber.msg"
    "msg/WheelBaseVehicle.msg"
    "msg/WrongWayDrivingSubCauseCode.msg"
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
