cmake_minimum_required(VERSION 3.5)
project(etsi_its_cam_ts_msgs)

find_package(ros_environment REQUIRED QUIET)
set(ROS_VERSION $ENV{ROS_VERSION})

# === ROS 2 (AMENT) ============================================================
if(${ROS_VERSION} EQUAL 2)

  find_package(ament_cmake REQUIRED)
  find_package(rosidl_default_generators REQUIRED)

  set(msg_files
    "msg/AccelerationComponent.msg"
    "msg/AccelerationConfidence.msg"
    "msg/AccelerationControl.msg"
    "msg/AccelerationValue.msg"
    "msg/AccidentSubCauseCode.msg"
    "msg/AdverseWeatherConditionAdhesionSubCauseCode.msg"
    "msg/AdverseWeatherConditionExtremeWeatherConditionSubCauseCode.msg"
    "msg/AdverseWeatherConditionPrecipitationSubCauseCode.msg"
    "msg/AdverseWeatherConditionVisibilitySubCauseCode.msg"
    "msg/Altitude.msg"
    "msg/AltitudeConfidence.msg"
    "msg/AltitudeValue.msg"
    "msg/BasicContainer.msg"
    "msg/BasicVehicleContainerHighFrequency.msg"
    "msg/BasicVehicleContainerLowFrequency.msg"
    "msg/CAM.msg"
    "msg/CamParameters.msg"
    "msg/CamPayload.msg"
    "msg/CauseCodeChoice.msg"
    "msg/CauseCodeV2.msg"
    "msg/CenDsrcTollingZone.msg"
    "msg/ClosedLanes.msg"
    "msg/CollisionRiskSubCauseCode.msg"
    "msg/Curvature.msg"
    "msg/CurvatureCalculationMode.msg"
    "msg/CurvatureConfidence.msg"
    "msg/CurvatureValue.msg"
    "msg/DangerousEndOfQueueSubCauseCode.msg"
    "msg/DangerousGoodsBasic.msg"
    "msg/DangerousGoodsContainer.msg"
    "msg/DangerousSituationSubCauseCode.msg"
    "msg/DeltaAltitude.msg"
    "msg/DeltaLatitude.msg"
    "msg/DeltaLongitude.msg"
    "msg/DeltaReferencePosition.msg"
    "msg/DriveDirection.msg"
    "msg/DrivingLaneStatus.msg"
    "msg/EmbarkationStatus.msg"
    "msg/EmergencyContainer.msg"
    "msg/EmergencyPriority.msg"
    "msg/EmergencyVehicleApproachingSubCauseCode.msg"
    "msg/ExteriorLights.msg"
    "msg/GenerationDeltaTime.msg"
    "msg/HardShoulderStatus.msg"
    "msg/HazardousLocationAnimalOnTheRoadSubCauseCode.msg"
    "msg/HazardousLocationDangerousCurveSubCauseCode.msg"
    "msg/HazardousLocationObstacleOnTheRoadSubCauseCode.msg"
    "msg/HazardousLocationSurfaceConditionSubCauseCode.msg"
    "msg/Heading.msg"
    "msg/HeadingConfidence.msg"
    "msg/HeadingValue.msg"
    "msg/HighFrequencyContainer.msg"
    "msg/HumanPresenceOnTheRoadSubCauseCode.msg"
    "msg/HumanProblemSubCauseCode.msg"
    "msg/ImpassabilitySubCauseCode.msg"
    "msg/ItsPduHeader.msg"
    "msg/LanePosition.msg"
    "msg/Latitude.msg"
    "msg/LightBarSirenInUse.msg"
    "msg/Longitude.msg"
    "msg/LowFrequencyContainer.msg"
    "msg/MessageId.msg"
    "msg/OrdinalNumber1B.msg"
    "msg/Path.msg"
    "msg/PathDeltaTime.msg"
    "msg/PathPoint.msg"
    "msg/PerformanceClass.msg"
    "msg/PositionConfidenceEllipse.msg"
    "msg/PostCrashSubCauseCode.msg"
    "msg/ProtectedCommunicationZone.msg"
    "msg/ProtectedCommunicationZonesRSU.msg"
    "msg/ProtectedZoneId.msg"
    "msg/ProtectedZoneRadius.msg"
    "msg/ProtectedZoneType.msg"
    "msg/PtActivation.msg"
    "msg/PtActivationData.msg"
    "msg/PtActivationType.msg"
    "msg/PublicTransportContainer.msg"
    "msg/RSUContainerHighFrequency.msg"
    "msg/RailwayLevelCrossingSubCauseCode.msg"
    "msg/ReferencePositionWithConfidence.msg"
    "msg/RescueAndRecoveryWorkInProgressSubCauseCode.msg"
    "msg/RescueContainer.msg"
    "msg/RoadWorksContainerBasic.msg"
    "msg/RoadworksSubCauseCode.msg"
    "msg/SafetyCarContainer.msg"
    "msg/SemiAxisLength.msg"
    "msg/SignalViolationSubCauseCode.msg"
    "msg/SlowVehicleSubCauseCode.msg"
    "msg/SpecialTransportContainer.msg"
    "msg/SpecialTransportType.msg"
    "msg/SpecialVehicleContainer.msg"
    "msg/Speed.msg"
    "msg/SpeedConfidence.msg"
    "msg/SpeedLimit.msg"
    "msg/SpeedValue.msg"
    "msg/StationId.msg"
    "msg/StationaryVehicleSubCauseCode.msg"
    "msg/SteeringWheelAngle.msg"
    "msg/SteeringWheelAngleConfidence.msg"
    "msg/SteeringWheelAngleValue.msg"
    "msg/SubCauseCodeType.msg"
    "msg/TimestampIts.msg"
    "msg/TrafficConditionSubCauseCode.msg"
    "msg/TrafficParticipantType.msg"
    "msg/TrafficRule.msg"
    "msg/VehicleBreakdownSubCauseCode.msg"
    "msg/VehicleLength.msg"
    "msg/VehicleLengthConfidenceIndication.msg"
    "msg/VehicleLengthValue.msg"
    "msg/VehicleRole.msg"
    "msg/VehicleWidth.msg"
    "msg/Wgs84AngleValue.msg"
    "msg/WrongWayDrivingSubCauseCode.msg"
    "msg/YawRate.msg"
    "msg/YawRateConfidence.msg"
    "msg/YawRateValue.msg"
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
