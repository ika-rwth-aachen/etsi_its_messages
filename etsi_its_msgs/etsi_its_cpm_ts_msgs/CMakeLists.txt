cmake_minimum_required(VERSION 3.5)
project(etsi_its_cpm_ts_msgs)

find_package(ros_environment REQUIRED QUIET)
set(ROS_VERSION $ENV{ROS_VERSION})

# === ROS 2 (AMENT) ============================================================
if(${ROS_VERSION} EQUAL 2)

  find_package(ament_cmake REQUIRED)
  find_package(rosidl_default_generators REQUIRED)

  set(msg_files
    "msg/Acceleration3dWithConfidence.msg"
    "msg/AccelerationCartesian.msg"
    "msg/AccelerationComponent.msg"
    "msg/AccelerationConfidence.msg"
    "msg/AccelerationMagnitude.msg"
    "msg/AccelerationMagnitudeValue.msg"
    "msg/AccelerationPolarWithZ.msg"
    "msg/AccelerationValue.msg"
    "msg/Altitude.msg"
    "msg/AltitudeConfidence.msg"
    "msg/AltitudeValue.msg"
    "msg/AngleConfidence.msg"
    "msg/AngularSpeedConfidence.msg"
    "msg/CardinalNumber1B.msg"
    "msg/CardinalNumber3b.msg"
    "msg/CartesianAngle.msg"
    "msg/CartesianAngleValue.msg"
    "msg/CartesianAngularVelocityComponent.msg"
    "msg/CartesianAngularVelocityComponentValue.msg"
    "msg/CartesianCoordinate.msg"
    "msg/CartesianCoordinateLarge.msg"
    "msg/CartesianCoordinateSmall.msg"
    "msg/CartesianCoordinateWithConfidence.msg"
    "msg/CartesianPosition3d.msg"
    "msg/CartesianPosition3dWithConfidence.msg"
    "msg/CircularShape.msg"
    "msg/CollectivePerceptionMessage.msg"
    "msg/ConfidenceLevel.msg"
    "msg/ConstraintWrappedCpmContainers.msg"
    "msg/CoordinateConfidence.msg"
    "msg/CorrelationCellValue.msg"
    "msg/CorrelationColumn.msg"
    "msg/CpmContainerId.msg"
    "msg/CpmPayload.msg"
    "msg/DeltaTimeMilliSecondSigned.msg"
    "msg/EllipticalShape.msg"
    "msg/EulerAnglesWithConfidence.msg"
    "msg/HeadingValue.msg"
    "msg/Identifier1B.msg"
    "msg/Identifier2B.msg"
    "msg/IntersectionReferenceId.msg"
    "msg/ItsPduHeader.msg"
    "msg/Latitude.msg"
    "msg/Longitude.msg"
    "msg/LongitudinalLanePosition.msg"
    "msg/LongitudinalLanePositionConfidence.msg"
    "msg/LongitudinalLanePositionValue.msg"
    "msg/LowerTriangularPositiveSemidefiniteMatrices.msg"
    "msg/LowerTriangularPositiveSemidefiniteMatrix.msg"
    "msg/LowerTriangularPositiveSemidefiniteMatrixColumns.msg"
    "msg/ManagementContainer.msg"
    "msg/MapPosition.msg"
    "msg/MapReference.msg"
    "msg/MatrixIncludedComponents.msg"
    "msg/MessageId.msg"
    "msg/MessageRateHz.msg"
    "msg/MessageRateRange.msg"
    "msg/MessageSegmentationInfo.msg"
    "msg/ObjectClass.msg"
    "msg/ObjectClassDescription.msg"
    "msg/ObjectClassWithConfidence.msg"
    "msg/ObjectDimension.msg"
    "msg/ObjectDimensionConfidence.msg"
    "msg/ObjectDimensionValue.msg"
    "msg/ObjectPerceptionQuality.msg"
    "msg/OrdinalNumber1B.msg"
    "msg/OrdinalNumber3b.msg"
    "msg/OriginatingRsuContainer.msg"
    "msg/OriginatingVehicleContainer.msg"
    "msg/OtherSubClass.msg"
    "msg/PerceivedObject.msg"
    "msg/PerceivedObjectContainer.msg"
    "msg/PerceivedObjectIds.msg"
    "msg/PerceivedObjects.msg"
    "msg/PerceptionRegion.msg"
    "msg/PerceptionRegionContainer.msg"
    "msg/PolygonalShape.msg"
    "msg/PosConfidenceEllipse.msg"
    "msg/RadialShape.msg"
    "msg/RadialShapeDetails.msg"
    "msg/RadialShapes.msg"
    "msg/RadialShapesList.msg"
    "msg/RectangularShape.msg"
    "msg/ReferencePosition.msg"
    "msg/RoadSegmentReferenceId.msg"
    "msg/SemiAxisLength.msg"
    "msg/SensorInformation.msg"
    "msg/SensorInformationContainer.msg"
    "msg/SensorType.msg"
    "msg/SequenceOfCartesianPosition3d.msg"
    "msg/SequenceOfIdentifier1B.msg"
    "msg/Shape.msg"
    "msg/Speed.msg"
    "msg/SpeedConfidence.msg"
    "msg/SpeedValue.msg"
    "msg/StandardLength12b.msg"
    "msg/StandardLength1B.msg"
    "msg/StationId.msg"
    "msg/TimestampIts.msg"
    "msg/TrafficParticipantType.msg"
    "msg/TrailerData.msg"
    "msg/TrailerDataSet.msg"
    "msg/VehicleWidth.msg"
    "msg/Velocity3dWithConfidence.msg"
    "msg/VelocityCartesian.msg"
    "msg/VelocityComponent.msg"
    "msg/VelocityComponentValue.msg"
    "msg/VelocityPolarWithZ.msg"
    "msg/VruClusterInformation.msg"
    "msg/VruClusterProfiles.msg"
    "msg/VruProfileAndSubprofile.msg"
    "msg/VruSubProfileAnimal.msg"
    "msg/VruSubProfileBicyclist.msg"
    "msg/VruSubProfileMotorcyclist.msg"
    "msg/VruSubProfilePedestrian.msg"
    "msg/Wgs84Angle.msg"
    "msg/Wgs84AngleConfidence.msg"
    "msg/Wgs84AngleValue.msg"
    "msg/WrappedCpmContainer.msg"
    "msg/WrappedCpmContainers.msg"
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
