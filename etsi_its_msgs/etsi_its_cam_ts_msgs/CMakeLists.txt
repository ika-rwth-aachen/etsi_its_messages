cmake_minimum_required(VERSION 3.5)
project(etsi_its_cam_ts_msgs)

find_package(ros_environment REQUIRED QUIET)
set(ROS_VERSION $ENV{ROS_VERSION})

# === ROS 2 (AMENT) ============================================================
if(${ROS_VERSION} EQUAL 2)

  find_package(ament_cmake REQUIRED)
  find_package(rosidl_default_generators REQUIRED)

  set(msg_files
    "msg/SpeedValue.msg"
    "msg/HazardousLocationSurfaceConditionSubCauseCode.msg"
    "msg/SpecialTransportType.msg"
    "msg/DeltaAltitude.msg"
    "msg/CauseCodeV2.msg"
    "msg/VehicleRole.msg"
    "msg/DrivingLaneStatus.msg"
    "msg/DangerousGoodsContainer.msg"
    "msg/AccelerationValue.msg"
    "msg/CenDsrcTollingZone.msg"
    "msg/DeltaReferencePosition.msg"
    "msg/BasicVehicleContainerLowFrequency.msg"
    "msg/CurvatureConfidence.msg"
    "msg/SemiAxisLength.msg"
    "msg/ProtectedCommunicationZonesRSU.msg"
    "msg/ClosedLanes.msg"
    "msg/StationaryVehicleSubCauseCode.msg"
    "msg/SteeringWheelAngleValue.msg"
    "msg/RescueContainer.msg"
    "msg/AccelerationConfidence.msg"
    "msg/CamParameters.msg"
    "msg/AccelerationComponent.msg"
    "msg/OrdinalNumber1B.msg"
    "msg/VehicleLength.msg"
    "msg/HumanProblemSubCauseCode.msg"
    "msg/GenerationDeltaTime.msg"
    "msg/DangerousSituationSubCauseCode.msg"
    "msg/RailwayLevelCrossingSubCauseCode.msg"
    "msg/Heading.msg"
    "msg/ItsPduHeader.msg"
    "msg/RSUContainerHighFrequency.msg"
    "msg/PublicTransportContainer.msg"
    "msg/StationId.msg"
    "msg/ProtectedCommunicationZone.msg"
    "msg/HazardousLocationDangerousCurveSubCauseCode.msg"
    "msg/RescueAndRecoveryWorkInProgressSubCauseCode.msg"
    "msg/ProtectedZoneId.msg"
    "msg/PerformanceClass.msg"
    "msg/ProtectedZoneRadius.msg"
    "msg/Wgs84AngleValue.msg"
    "msg/AccelerationControl.msg"
    "msg/Curvature.msg"
    "msg/HeadingValue.msg"
    "msg/SubCauseCodeType.msg"
    "msg/PtActivationData.msg"
    "msg/SteeringWheelAngle.msg"
    "msg/AdverseWeatherConditionAdhesionSubCauseCode.msg"
    "msg/VehicleBreakdownSubCauseCode.msg"
    "msg/TrafficParticipantType.msg"
    "msg/SpeedConfidence.msg"
    "msg/LanePosition.msg"
    "msg/AltitudeConfidence.msg"
    "msg/TimestampIts.msg"
    "msg/SafetyCarContainer.msg"
    "msg/Path.msg"
    "msg/AdverseWeatherConditionVisibilitySubCauseCode.msg"
    "msg/WrongWayDrivingSubCauseCode.msg"
    "msg/AltitudeValue.msg"
    "msg/CurvatureValue.msg"
    "msg/ProtectedZoneType.msg"
    "msg/PathPoint.msg"
    "msg/EmbarkationStatus.msg"
    "msg/EmergencyPriority.msg"
    "msg/HighFrequencyContainer.msg"
    "msg/TrafficConditionSubCauseCode.msg"
    "msg/Speed.msg"
    "msg/AdverseWeatherConditionPrecipitationSubCauseCode.msg"
    "msg/ImpassabilitySubCauseCode.msg"
    "msg/DeltaLatitude.msg"
    "msg/HazardousLocationObstacleOnTheRoadSubCauseCode.msg"
    "msg/RoadWorksContainerBasic.msg"
    "msg/AccidentSubCauseCode.msg"
    "msg/YawRate.msg"
    "msg/BasicVehicleContainerHighFrequency.msg"
    "msg/CurvatureCalculationMode.msg"
    "msg/CauseCodeChoice.msg"
    "msg/LightBarSirenInUse.msg"
    "msg/Longitude.msg"
    "msg/VehicleLengthValue.msg"
    "msg/PtActivation.msg"
    "msg/CollisionRiskSubCauseCode.msg"
    "msg/SpecialVehicleContainer.msg"
    "msg/DriveDirection.msg"
    "msg/SlowVehicleSubCauseCode.msg"
    "msg/SpecialTransportContainer.msg"
    "msg/SteeringWheelAngleConfidence.msg"
    "msg/TrafficRule.msg"
    "msg/BasicContainer.msg"
    "msg/DeltaLongitude.msg"
    "msg/ReferencePositionWithConfidence.msg"
    "msg/HardShoulderStatus.msg"
    "msg/Latitude.msg"
    "msg/VehicleWidth.msg"
    "msg/ExteriorLights.msg"
    "msg/Altitude.msg"
    "msg/CamPayload.msg"
    "msg/EmergencyContainer.msg"
    "msg/DangerousEndOfQueueSubCauseCode.msg"
    "msg/LowFrequencyContainer.msg"
    "msg/AdverseWeatherConditionExtremeWeatherConditionSubCauseCode.msg"
    "msg/PositionConfidenceEllipse.msg"
    "msg/RoadworksSubCauseCode.msg"
    "msg/DangerousGoodsBasic.msg"
    "msg/SpeedLimit.msg"
    "msg/YawRateValue.msg"
    "msg/PtActivationType.msg"
    "msg/PathDeltaTime.msg"
    "msg/MessageId.msg"
    "msg/HazardousLocationAnimalOnTheRoadSubCauseCode.msg"
    "msg/EmergencyVehicleApproachingSubCauseCode.msg"
    "msg/HumanPresenceOnTheRoadSubCauseCode.msg"
    "msg/VehicleLengthConfidenceIndication.msg"
    "msg/YawRateConfidence.msg"
    "msg/SignalViolationSubCauseCode.msg"
    "msg/PostCrashSubCauseCode.msg"
    "msg/HeadingConfidence.msg"
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
