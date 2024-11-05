cmake_minimum_required(VERSION 3.5)
project(etsi_its_cpm_ts_msgs)

find_package(ros_environment REQUIRED QUIET)
set(ROS_VERSION $ENV{ROS_VERSION})

# === ROS 2 (AMENT) ============================================================
if(${ROS_VERSION} EQUAL 2)

  find_package(ament_cmake REQUIRED)
  find_package(rosidl_default_generators REQUIRED)

  set(msg_files
    "msg/MessageSegmentationInfo.msg"
    "msg/Shape.msg"
    "msg/Velocity3dWithConfidence.msg"
    "msg/RadialShapes.msg"
    "msg/Wgs84Angle.msg"
    "msg/PerceivedObject.msg"
    "msg/AccelerationMagnitudeValue.msg"
    "msg/LongitudinalLanePositionConfidence.msg"
    "msg/StationId.msg"
    "msg/VelocityComponent.msg"
    "msg/ObjectDimensionValue.msg"
    "msg/Identifier2B.msg"
    "msg/VruSubProfileMotorcyclist.msg"
    "msg/CartesianAngleValue.msg"
    "msg/VelocityPolarWithZ.msg"
    "msg/VelocityComponentValue.msg"
    "msg/OrdinalNumber1B.msg"
    "msg/CartesianAngularVelocityComponentValue.msg"
    "msg/OriginatingVehicleContainer.msg"
    "msg/Speed.msg"
    "msg/ObjectDimension.msg"
    "msg/Latitude.msg"
    "msg/AltitudeConfidence.msg"
    "msg/OtherSubClass.msg"
    "msg/MapPosition.msg"
    "msg/MapReference.msg"
    "msg/SensorInformationContainer.msg"
    "msg/CartesianPosition3dWithConfidence.msg"
    "msg/TrafficParticipantType.msg"
    "msg/SemiAxisLength.msg"
    "msg/CardinalNumber3b.msg"
    "msg/VruClusterProfiles.msg"
    "msg/PerceivedObjectContainer.msg"
    "msg/VruProfileAndSubprofile.msg"
    "msg/Acceleration3dWithConfidence.msg"
    "msg/RoadSegmentReferenceId.msg"
    "msg/LowerTriangularPositiveSemidefiniteMatrix.msg"
    "msg/CartesianAngle.msg"
    "msg/CpmPayload.msg"
    "msg/ObjectClass.msg"
    "msg/AltitudeValue.msg"
    "msg/RadialShape.msg"
    "msg/VruSubProfileBicyclist.msg"
    "msg/EulerAnglesWithConfidence.msg"
    "msg/AccelerationMagnitude.msg"
    "msg/ConstraintWrappedCpmContainers.msg"
    "msg/CardinalNumber1B.msg"
    "msg/CircularShape.msg"
    "msg/SensorType.msg"
    "msg/CartesianCoordinateSmall.msg"
    "msg/VruSubProfileAnimal.msg"
    "msg/ManagementContainer.msg"
    "msg/TimestampIts.msg"
    "msg/StandardLength1B.msg"
    "msg/ReferencePosition.msg"
    "msg/WrappedCpmContainer.msg"
    "msg/PolygonalShape.msg"
    "msg/DeltaTimeMilliSecondSigned.msg"
    "msg/PerceivedObjects.msg"
    "msg/PerceivedObjectIds.msg"
    "msg/ObjectDimensionConfidence.msg"
    "msg/LongitudinalLanePositionValue.msg"
    "msg/WrappedCpmContainers.msg"
    "msg/StandardLength12b.msg"
    "msg/TrailerDataSet.msg"
    "msg/EllipticalShape.msg"
    "msg/AccelerationValue.msg"
    "msg/CartesianCoordinateWithConfidence.msg"
    "msg/ObjectClassWithConfidence.msg"
    "msg/LongitudinalLanePosition.msg"
    "msg/RadialShapeDetails.msg"
    "msg/SequenceOfIdentifier1B.msg"
    "msg/VruSubProfilePedestrian.msg"
    "msg/ItsPduHeader.msg"
    "msg/HeadingValue.msg"
    "msg/SensorInformation.msg"
    "msg/MatrixIncludedComponents.msg"
    "msg/CartesianAngularVelocityComponent.msg"
    "msg/RectangularShape.msg"
    "msg/CartesianCoordinateLarge.msg"
    "msg/SpeedValue.msg"
    "msg/AccelerationCartesian.msg"
    "msg/AngularSpeedConfidence.msg"
    "msg/TrailerData.msg"
    "msg/ObjectClassDescription.msg"
    "msg/CpmContainerId.msg"
    "msg/RadialShapesList.msg"
    "msg/VelocityCartesian.msg"
    "msg/PerceptionRegion.msg"
    "msg/Longitude.msg"
    "msg/LowerTriangularPositiveSemidefiniteMatrices.msg"
    "msg/ConfidenceLevel.msg"
    "msg/CorrelationCellValue.msg"
    "msg/PosConfidenceEllipse.msg"
    "msg/OriginatingRsuContainer.msg"
    "msg/AccelerationComponent.msg"
    "msg/PerceptionRegionContainer.msg"
    "msg/IntersectionReferenceId.msg"
    "msg/Identifier1B.msg"
    "msg/VehicleWidth.msg"
    "msg/Wgs84AngleValue.msg"
    "msg/VruClusterInformation.msg"
    "msg/CartesianCoordinate.msg"
    "msg/AngleConfidence.msg"
    "msg/SpeedConfidence.msg"
    "msg/ObjectPerceptionQuality.msg"
    "msg/CorrelationColumn.msg"
    "msg/CoordinateConfidence.msg"
    "msg/MessageRateHz.msg"
    "msg/MessageRateRange.msg"
    "msg/MessageId.msg"
    "msg/AccelerationConfidence.msg"
    "msg/OrdinalNumber3b.msg"
    "msg/LowerTriangularPositiveSemidefiniteMatrixColumns.msg"
    "msg/AccelerationPolarWithZ.msg"
    "msg/SequenceOfCartesianPosition3d.msg"
    "msg/CartesianPosition3d.msg"
    "msg/Wgs84AngleConfidence.msg"
    "msg/CpmContainers.msg"
    "msg/Altitude.msg"
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
