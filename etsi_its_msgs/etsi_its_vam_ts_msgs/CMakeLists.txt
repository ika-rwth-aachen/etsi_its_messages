cmake_minimum_required(VERSION 3.5)
project(etsi_its_vam_ts_msgs)

find_package(ros_environment REQUIRED QUIET)
set(ROS_VERSION $ENV{ROS_VERSION})

# === ROS 2 (AMENT) ============================================================
if(${ROS_VERSION} EQUAL 2)

  find_package(ament_cmake REQUIRED)
  find_package(rosidl_default_generators REQUIRED)

  set(msg_files
    "msg/AccelerationChange.msg"
    "msg/AccelerationChangeIndication.msg"
    "msg/AccelerationConfidence.msg"
    "msg/Altitude.msg"
    "msg/AltitudeConfidence.msg"
    "msg/AltitudeValue.msg"
    "msg/AngleConfidence.msg"
    "msg/BasicContainer.msg"
    "msg/CardinalNumber1B.msg"
    "msg/CartesianAngle.msg"
    "msg/CartesianAngleValue.msg"
    "msg/CartesianCoordinate.msg"
    "msg/CartesianCoordinateSmall.msg"
    "msg/CartesianPosition3d.msg"
    "msg/CircularShape.msg"
    "msg/ClusterBreakupInfo.msg"
    "msg/ClusterBreakupReason.msg"
    "msg/ClusterJoinInfo.msg"
    "msg/ClusterLeaveInfo.msg"
    "msg/ClusterLeaveReason.msg"
    "msg/Curvature.msg"
    "msg/CurvatureCalculationMode.msg"
    "msg/CurvatureConfidence.msg"
    "msg/CurvatureValue.msg"
    "msg/DeltaAltitude.msg"
    "msg/DeltaLatitude.msg"
    "msg/DeltaLongitude.msg"
    "msg/DeltaReferencePosition.msg"
    "msg/DeltaTimeQuarterSecond.msg"
    "msg/DeltaTimeTenthOfSecond.msg"
    "msg/EllipticalShape.msg"
    "msg/ExteriorLights.msg"
    "msg/GeneralizedLanePosition.msg"
    "msg/GenerationDeltaTime.msg"
    "msg/HeadingChangeIndication.msg"
    "msg/HeadingValue.msg"
    "msg/Identifier1B.msg"
    "msg/Identifier2B.msg"
    "msg/IntersectionReferenceId.msg"
    "msg/ItsPduHeader.msg"
    "msg/ItsPduHeaderVam.msg"
    "msg/LanePosition.msg"
    "msg/LanePositionAndType.msg"
    "msg/LaneType.msg"
    "msg/LateralAcceleration.msg"
    "msg/LateralAccelerationValue.msg"
    "msg/Latitude.msg"
    "msg/Longitude.msg"
    "msg/LongitudinalAcceleration.msg"
    "msg/LongitudinalAccelerationValue.msg"
    "msg/LongitudinalLanePosition.msg"
    "msg/LongitudinalLanePositionConfidence.msg"
    "msg/LongitudinalLanePositionValue.msg"
    "msg/MapPosition.msg"
    "msg/MapReference.msg"
    "msg/MessageId.msg"
    "msg/OrdinalNumber1B.msg"
    "msg/PathDeltaTime.msg"
    "msg/PathHistory.msg"
    "msg/PathPoint.msg"
    "msg/PathPointPredicted.msg"
    "msg/PathPredicted.msg"
    "msg/PolygonalShape.msg"
    "msg/PosConfidenceEllipse.msg"
    "msg/PositionConfidenceEllipse.msg"
    "msg/RadialShape.msg"
    "msg/RadialShapeDetails.msg"
    "msg/RadialShapes.msg"
    "msg/RadialShapesList.msg"
    "msg/RectangularShape.msg"
    "msg/ReferencePositionWithConfidence.msg"
    "msg/RoadSegmentReferenceId.msg"
    "msg/SafeDistanceIndication.msg"
    "msg/SafeDistanceIndicator.msg"
    "msg/SemiAxisLength.msg"
    "msg/SequenceOfCartesianPosition3d.msg"
    "msg/SequenceOfSafeDistanceIndication.msg"
    "msg/SequenceOfTrajectoryInterceptionIndication.msg"
    "msg/Shape.msg"
    "msg/Speed.msg"
    "msg/SpeedConfidence.msg"
    "msg/SpeedValue.msg"
    "msg/StabilityChangeIndication.msg"
    "msg/StabilityLossProbability.msg"
    "msg/StandardLength12b.msg"
    "msg/StationId.msg"
    "msg/TrafficIslandPosition.msg"
    "msg/TrafficParticipantType.msg"
    "msg/TrajectoryInterceptionConfidence.msg"
    "msg/TrajectoryInterceptionIndication.msg"
    "msg/TrajectoryInterceptionProbability.msg"
    "msg/TurningDirection.msg"
    "msg/VAM.msg"
    "msg/VamParameters.msg"
    "msg/VerticalAcceleration.msg"
    "msg/VerticalAccelerationValue.msg"
    "msg/VruAwareness.msg"
    "msg/VruClusterInformation.msg"
    "msg/VruClusterInformationContainer.msg"
    "msg/VruClusterOperationContainer.msg"
    "msg/VruClusterProfiles.msg"
    "msg/VruDeviceUsage.msg"
    "msg/VruEnvironment.msg"
    "msg/VruExteriorLights.msg"
    "msg/VruHighFrequencyContainer.msg"
    "msg/VruLowFrequencyContainer.msg"
    "msg/VruMotionPredictionContainer.msg"
    "msg/VruMovementControl.msg"
    "msg/VruProfileAndSubprofile.msg"
    "msg/VruSizeClass.msg"
    "msg/VruSpecificExteriorLights.msg"
    "msg/VruSubProfileAnimal.msg"
    "msg/VruSubProfileBicyclist.msg"
    "msg/VruSubProfileMotorcyclist.msg"
    "msg/VruSubProfilePedestrian.msg"
    "msg/Wgs84Angle.msg"
    "msg/Wgs84AngleConfidence.msg"
    "msg/Wgs84AngleValue.msg"
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
