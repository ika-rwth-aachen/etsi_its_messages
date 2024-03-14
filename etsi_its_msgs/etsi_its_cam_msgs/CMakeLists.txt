cmake_minimum_required(VERSION 3.5)
project(etsi_its_cam_msgs)

find_package(ros_environment REQUIRED QUIET)
set(ROS_VERSION $ENV{ROS_VERSION})

# === ROS 2 (AMENT) ============================================================
if(${ROS_VERSION} EQUAL 2)

  find_package(ament_cmake REQUIRED)
  find_package(rosidl_default_generators REQUIRED)

  set(msg_files
    "msg/AccelerationConfidence.msg"
    "msg/AccelerationControl.msg"
    "msg/AccidentSubCauseCode.msg"
    "msg/ActionID.msg"
    "msg/AdverseWeatherConditionAdhesionSubCauseCode.msg"
    "msg/AdverseWeatherConditionExtremeWeatherConditionSubCauseCode.msg"
    "msg/AdverseWeatherConditionPrecipitationSubCauseCode.msg"
    "msg/AdverseWeatherConditionVisibilitySubCauseCode.msg"
    "msg/AltitudeConfidence.msg"
    "msg/Altitude.msg"
    "msg/AltitudeValue.msg"
    "msg/BasicContainer.msg"
    "msg/BasicVehicleContainerHighFrequency.msg"
    "msg/BasicVehicleContainerLowFrequency.msg"
    "msg/CAM.msg"
    "msg/CamParameters.msg"
    "msg/CauseCode.msg"
    "msg/CauseCodeType.msg"
    "msg/CenDsrcTollingZoneID.msg"
    "msg/CenDsrcTollingZone.msg"
    "msg/ClosedLanes.msg"
    "msg/CollisionRiskSubCauseCode.msg"
    "msg/CoopAwareness.msg"
    "msg/CurvatureCalculationMode.msg"
    "msg/CurvatureConfidence.msg"
    "msg/Curvature.msg"
    "msg/CurvatureValue.msg"
    "msg/DangerousEndOfQueueSubCauseCode.msg"
    "msg/DangerousGoodsBasic.msg"
    "msg/DangerousGoodsContainer.msg"
    "msg/DangerousGoodsExtended.msg"
    "msg/DangerousSituationSubCauseCode.msg"
    "msg/DeltaAltitude.msg"
    "msg/DeltaLatitude.msg"
    "msg/DeltaLongitude.msg"
    "msg/DeltaReferencePosition.msg"
    "msg/DigitalMap.msg"
    "msg/DriveDirection.msg"
    "msg/DrivingLaneStatus.msg"
    "msg/EmbarkationStatus.msg"
    "msg/EmergencyContainer.msg"
    "msg/EmergencyPriority.msg"
    "msg/EmergencyVehicleApproachingSubCauseCode.msg"
    "msg/EnergyStorageType.msg"
    "msg/EventHistory.msg"
    "msg/EventPoint.msg"
    "msg/ExteriorLights.msg"
    "msg/GenerationDeltaTime.msg"
    "msg/HardShoulderStatus.msg"
    "msg/HazardousLocationAnimalOnTheRoadSubCauseCode.msg"
    "msg/HazardousLocationDangerousCurveSubCauseCode.msg"
    "msg/HazardousLocationObstacleOnTheRoadSubCauseCode.msg"
    "msg/HazardousLocationSurfaceConditionSubCauseCode.msg"
    "msg/HeadingConfidence.msg"
    "msg/Heading.msg"
    "msg/HeadingValue.msg"
    "msg/HeightLonCarr.msg"
    "msg/HighFrequencyContainer.msg"
    "msg/HumanPresenceOnTheRoadSubCauseCode.msg"
    "msg/HumanProblemSubCauseCode.msg"
    "msg/InformationQuality.msg"
    "msg/ItineraryPath.msg"
    "msg/ItsPduHeader.msg"
    "msg/LanePosition.msg"
    "msg/LateralAcceleration.msg"
    "msg/LateralAccelerationValue.msg"
    "msg/Latitude.msg"
    "msg/LightBarSirenInUse.msg"
    "msg/Longitude.msg"
    "msg/LongitudinalAcceleration.msg"
    "msg/LongitudinalAccelerationValue.msg"
    "msg/LowFrequencyContainer.msg"
    "msg/NumberOfOccupants.msg"
    "msg/OpeningDaysHours.msg"
    "msg/PathDeltaTime.msg"
    "msg/PathHistory.msg"
    "msg/PathPoint.msg"
    "msg/PerformanceClass.msg"
    "msg/PhoneNumber.msg"
    "msg/PosCentMass.msg"
    "msg/PosConfidenceEllipse.msg"
    "msg/PosFrontAx.msg"
    "msg/PositioningSolutionType.msg"
    "msg/PositionOfOccupants.msg"
    "msg/PositionOfPillars.msg"
    "msg/PosLonCarr.msg"
    "msg/PosPillar.msg"
    "msg/PostCrashSubCauseCode.msg"
    "msg/ProtectedCommunicationZone.msg"
    "msg/ProtectedCommunicationZonesRSU.msg"
    "msg/ProtectedZoneID.msg"
    "msg/ProtectedZoneRadius.msg"
    "msg/ProtectedZoneType.msg"
    "msg/PtActivationData.msg"
    "msg/PtActivation.msg"
    "msg/PtActivationType.msg"
    "msg/PublicTransportContainer.msg"
    "msg/ReferencePosition.msg"
    "msg/RelevanceDistance.msg"
    "msg/RelevanceTrafficDirection.msg"
    "msg/RequestResponseIndication.msg"
    "msg/RescueAndRecoveryWorkInProgressSubCauseCode.msg"
    "msg/RescueContainer.msg"
    "msg/RestrictedTypes.msg"
    "msg/RoadType.msg"
    "msg/RoadWorksContainerBasic.msg"
    "msg/RoadworksSubCauseCode.msg"
    "msg/RSUContainerHighFrequency.msg"
    "msg/SafetyCarContainer.msg"
    "msg/SemiAxisLength.msg"
    "msg/SequenceNumber.msg"
    "msg/SignalViolationSubCauseCode.msg"
    "msg/SlowVehicleSubCauseCode.msg"
    "msg/SpecialTransportContainer.msg"
    "msg/SpecialTransportType.msg"
    "msg/SpecialVehicleContainer.msg"
    "msg/SpeedConfidence.msg"
    "msg/SpeedLimit.msg"
    "msg/Speed.msg"
    "msg/SpeedValue.msg"
    "msg/StationarySince.msg"
    "msg/StationaryVehicleSubCauseCode.msg"
    "msg/StationID.msg"
    "msg/StationType.msg"
    "msg/SteeringWheelAngleConfidence.msg"
    "msg/SteeringWheelAngle.msg"
    "msg/SteeringWheelAngleValue.msg"
    "msg/SubCauseCodeType.msg"
    "msg/Temperature.msg"
    "msg/TimestampIts.msg"
    "msg/Traces.msg"
    "msg/TrafficConditionSubCauseCode.msg"
    "msg/TrafficRule.msg"
    "msg/TransmissionInterval.msg"
    "msg/TurningRadius.msg"
    "msg/ValidityDuration.msg"
    "msg/VDS.msg"
    "msg/VehicleBreakdownSubCauseCode.msg"
    "msg/VehicleIdentification.msg"
    "msg/VehicleLengthConfidenceIndication.msg"
    "msg/VehicleLength.msg"
    "msg/VehicleLengthValue.msg"
    "msg/VehicleMass.msg"
    "msg/VehicleRole.msg"
    "msg/VehicleWidth.msg"
    "msg/VerticalAcceleration.msg"
    "msg/VerticalAccelerationValue.msg"
    "msg/WheelBaseVehicle.msg"
    "msg/WMInumber.msg"
    "msg/WrongWayDrivingSubCauseCode.msg"
    "msg/YawRateConfidence.msg"
    "msg/YawRate.msg"
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
