cmake_minimum_required(VERSION 3.5)
project(etsi_its_cam_msgs)

find_package(ros_environment REQUIRED QUIET)
set(ROS_VERSION $ENV{ROS_VERSION})

# === ROS 2 (AMENT) ============================================================
if(${ROS_VERSION} EQUAL 2)

  find_package(ament_cmake REQUIRED)
  find_package(rosidl_default_generators REQUIRED)

  set(msg_files
    "msg/CauseCodeType.msg"
    "msg/PublicTransportContainer.msg"
    "msg/VehicleLengthValue.msg"
    "msg/EmergencyContainer.msg"
    "msg/ProtectedCommunicationZone.msg"
    "msg/SpeedValue.msg"
    "msg/RoadworksSubCauseCode.msg"
    "msg/SpecialVehicleContainer.msg"
    "msg/Latitude.msg"
    "msg/ProtectedCommunicationZonesRSU.msg"
    "msg/LongitudinalAcceleration.msg"
    "msg/DeltaReferencePosition.msg"
    "msg/SteeringWheelAngleConfidence.msg"
    "msg/PathHistory.msg"
    "msg/DangerousGoodsContainer.msg"
    "msg/EmbarkationStatus.msg"
    "msg/PtActivationData.msg"
    "msg/StationID.msg"
    "msg/VehicleLength.msg"
    "msg/PathDeltaTime.msg"
    "msg/AccelerationControl.msg"
    "msg/VerticalAccelerationValue.msg"
    "msg/RSUContainerHighFrequency.msg"
    "msg/BasicVehicleContainerLowFrequency.msg"
    "msg/ProtectedZoneID.msg"
    "msg/DeltaLatitude.msg"
    "msg/VerticalAcceleration.msg"
    "msg/RescueContainer.msg"
    "msg/VehicleRole.msg"
    "msg/PtActivation.msg"
    "msg/YawRateConfidence.msg"
    "msg/VehicleLengthConfidenceIndication.msg"
    "msg/Longitude.msg"
    "msg/PtActivationType.msg"
    "msg/CenDsrcTollingZone.msg"
    "msg/TrafficRule.msg"
    "msg/CenDsrcTollingZoneID.msg"
    "msg/DrivingLaneStatus.msg"
    "msg/DeltaLongitude.msg"
    "msg/Altitude.msg"
    "msg/CauseCode.msg"
    "msg/SteeringWheelAngleValue.msg"
    "msg/HeadingConfidence.msg"
    "msg/YawRateValue.msg"
    "msg/LateralAcceleration.msg"
    "msg/StationType.msg"
    "msg/ProtectedZoneType.msg"
    "msg/EmergencyPriority.msg"
    "msg/HeadingValue.msg"
    "msg/SpeedLimit.msg"
    "msg/SteeringWheelAngle.msg"
    "msg/ExteriorLights.msg"
    "msg/LongitudinalAccelerationValue.msg"
    "msg/DangerousGoodsBasic.msg"
    "msg/SpecialTransportContainer.msg"
    "msg/SpeedConfidence.msg"
    "msg/BasicContainer.msg"
    "msg/ClosedLanes.msg"
    "msg/PosConfidenceEllipse.msg"
    "msg/SafetyCarContainer.msg"
    "msg/ProtectedZoneRadius.msg"
    "msg/LanePosition.msg"
    "msg/AltitudeValue.msg"
    "msg/PerformanceClass.msg"
    "msg/HighFrequencyContainer.msg"
    "msg/CurvatureValue.msg"
    "msg/YawRate.msg"
    "msg/LightBarSirenInUse.msg"
    "msg/SemiAxisLength.msg"
    "msg/PathPoint.msg"
    "msg/Speed.msg"
    "msg/ReferencePosition.msg"
    "msg/LateralAccelerationValue.msg"
    "msg/VehicleWidth.msg"
    "msg/CurvatureConfidence.msg"
    "msg/Curvature.msg"
    "msg/SpecialTransportType.msg"
    "msg/Heading.msg"
    "msg/BasicVehicleContainerHighFrequency.msg"
    "msg/DriveDirection.msg"
    "msg/CamParameters.msg"
    "msg/AccelerationConfidence.msg"
    "msg/TimestampIts.msg"
    "msg/SubCauseCodeType.msg"
    "msg/CoopAwareness.msg"
    "msg/ItsPduHeader.msg"
    "msg/DeltaAltitude.msg"
    "msg/RoadWorksContainerBasic.msg"
    "msg/AltitudeConfidence.msg"
    "msg/GenerationDeltaTime.msg"
    "msg/LowFrequencyContainer.msg"
    "msg/CurvatureCalculationMode.msg"
    "msg/HardShoulderStatus.msg"
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
