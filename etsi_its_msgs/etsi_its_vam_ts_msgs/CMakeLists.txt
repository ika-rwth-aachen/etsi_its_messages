cmake_minimum_required(VERSION 3.5)
project(etsi_its_vam_ts_msgs)

find_package(ros_environment REQUIRED QUIET)
set(ROS_VERSION $ENV{ROS_VERSION})

# === ROS 2 (AMENT) ============================================================
if(${ROS_VERSION} EQUAL 2)

  find_package(ament_cmake REQUIRED)
  find_package(rosidl_default_generators REQUIRED)

  set(msg_files
    "msg/MessageId.msg"
    "msg/ItsPduHeaderVam.msg"
    "msg/VruSubProfilePedestrian.msg"
    "msg/LongitudinalAcceleration.msg"
    "msg/HeadingChangeIndication.msg"
    "msg/DeltaTimeTenthOfSecond.msg"
    "msg/SpeedConfidence.msg"
    "msg/VruClusterInformationContainer.msg"
    "msg/VerticalAccelerationValue.msg"
    "msg/LanePosition.msg"
    "msg/VruSubProfileAnimal.msg"
    "msg/Wgs84AngleConfidence.msg"
    "msg/CartesianCoordinate.msg"
    "msg/CartesianAngleValue.msg"
    "msg/LongitudinalLanePositionValue.msg"
    "msg/CurvatureValue.msg"
    "msg/AltitudeValue.msg"
    "msg/TrajectoryInterceptionConfidence.msg"
    "msg/VruHighFrequencyContainer.msg"
    "msg/LongitudinalAccelerationValue.msg"
    "msg/LaneType.msg"
    "msg/TrajectoryInterceptionProbability.msg"
    "msg/YawRateValue.msg"
    "msg/VruClusterOperationContainer.msg"
    "msg/PathHistory.msg"
    "msg/StandardLength12b.msg"
    "msg/SequenceOfCartesianPosition3d.msg"
    "msg/VruClusterProfiles.msg"
    "msg/ClusterBreakupReason.msg"
    "msg/ClusterBreakupInfo.msg"
    "msg/PathDeltaTime.msg"
    "msg/VruExteriorLights.msg"
    "msg/SafeDistanceIndication.msg"
    "msg/Shape.msg"
    "msg/StabilityChangeIndication.msg"
    "msg/VruSubProfileBicyclist.msg"
    "msg/Wgs84AngleValue.msg"
    "msg/SequenceOfSafeDistanceIndication.msg"
    "msg/AccelerationChange.msg"
    "msg/RectangularShape.msg"
    "msg/CardinalNumber1B.msg"
    "msg/VruAwareness.msg"
    "msg/LateralAcceleration.msg"
    "msg/RadialShape.msg"
    "msg/EllipticalShape.msg"
    "msg/HeadingValue.msg"
    "msg/RadialShapesList.msg"
    "msg/Longitude.msg"
    "msg/Curvature.msg"
    "msg/AngleConfidence.msg"
    "msg/CurvatureCalculationMode.msg"
    "msg/LateralAccelerationValue.msg"
    "msg/CartesianPosition3d.msg"
    "msg/GeneralizedLanePosition.msg"
    "msg/OrdinalNumber1B.msg"
    "msg/Altitude.msg"
    "msg/RadialShapeDetails.msg"
    "msg/VruMovementControl.msg"
    "msg/PathPredicted.msg"
    "msg/VruDeviceUsage.msg"
    "msg/LongitudinalLanePosition.msg"
    "msg/ItsPduHeader.msg"
    "msg/Identifier1B.msg"
    "msg/VruLowFrequencyContainer.msg"
    "msg/MapPosition.msg"
    "msg/ExteriorLights.msg"
    "msg/StabilityLossProbability.msg"
    "msg/AltitudeConfidence.msg"
    "msg/GenerationDeltaTime.msg"
    "msg/VruEnvironment.msg"
    "msg/SemiAxisLength.msg"
    "msg/PathPoint.msg"
    "msg/SpeedValue.msg"
    "msg/CartesianAngle.msg"
    "msg/LongitudinalLanePositionConfidence.msg"
    "msg/YawRate.msg"
    "msg/DeltaTimeQuarterSecond.msg"
    "msg/BasicContainer.msg"
    "msg/CircularShape.msg"
    "msg/PositionConfidenceEllipse.msg"
    "msg/TrafficParticipantType.msg"
    "msg/AccelerationConfidence.msg"
    "msg/MapReference.msg"
    "msg/SequenceOfTrajectoryInterceptionIndication.msg"
    "msg/LanePositionAndType.msg"
    "msg/VruMotionPredictionContainer.msg"
    "msg/ClusterJoinInfo.msg"
    "msg/VerticalAcceleration.msg"
    "msg/CurvatureConfidence.msg"
    "msg/PolygonalShape.msg"
    "msg/PosConfidenceEllipse.msg"
    "msg/VruSpecificExteriorLights.msg"
    "msg/VruSizeClass.msg"
    "msg/CartesianCoordinateSmall.msg"
    "msg/DeltaAltitude.msg"
    "msg/ClusterLeaveReason.msg"
    "msg/PathPointPredicted.msg"
    "msg/RoadSegmentReferenceId.msg"
    "msg/TrafficIslandPosition.msg"
    "msg/YawRateConfidence.msg"
    "msg/VruClusterInformation.msg"
    "msg/Speed.msg"
    "msg/SafeDistanceIndicator.msg"
    "msg/DeltaReferencePosition.msg"
    "msg/ClusterLeaveInfo.msg"
    "msg/Identifier2B.msg"
    "msg/VruProfileAndSubprofile.msg"
    "msg/DeltaLatitude.msg"
    "msg/StationId.msg"
    "msg/Wgs84Angle.msg"
    "msg/AccelerationChangeIndication.msg"
    "msg/Latitude.msg"
    "msg/RadialShapes.msg"
    "msg/TurningDirection.msg"
    "msg/DeltaLongitude.msg"
    "msg/TrajectoryInterceptionIndication.msg"
    "msg/ReferencePositionWithConfidence.msg"
    "msg/IntersectionReferenceId.msg"
    "msg/VruSubProfileMotorcyclist.msg"
    "msg/VamParameters.msg"
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
