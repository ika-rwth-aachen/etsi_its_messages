cmake_minimum_required(VERSION 3.0.2)
project(etsi_its_mapem_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder

add_message_files(
  DIRECTORY
    msg/MAPEM-PDU-Descriptions
  FILES
    MAPEM.msg
)

add_message_files(
  DIRECTORY
    msg/ITS-Container
  FILES
    AccelerationConfidence.msg
    AccelerationControl.msg
    AccidentSubCauseCode.msg
    ActionID.msg
    AdverseWeatherCondition_AdhesionSubCauseCode.msg
    AdverseWeatherCondition_ExtremeWeatherConditionSubCauseCode.msg
    AdverseWeatherCondition_PrecipitationSubCauseCode.msg
    AdverseWeatherCondition_VisibilitySubCauseCode.msg
    AltitudeConfidence.msg
    Altitude.msg
    AltitudeValue.msg
    CauseCode.msg
    CauseCodeType.msg
    CenDsrcTollingZoneID.msg
    CenDsrcTollingZone.msg
    ClosedLanes.msg
    CollisionRiskSubCauseCode.msg
    CurvatureCalculationMode.msg
    CurvatureConfidence.msg
    Curvature.msg
    CurvatureValue.msg
    DangerousEndOfQueueSubCauseCode.msg
    DangerousGoodsBasic.msg
    DangerousGoodsExtended.msg
    DangerousSituationSubCauseCode.msg
    DeltaAltitude.msg
    DeltaLatitude.msg
    DeltaLongitude.msg
    DeltaReferencePosition.msg
    DigitalMap.msg
    DriveDirection.msg
    DrivingLaneStatus.msg
    EmbarkationStatus.msg
    EmergencyPriority.msg
    EmergencyVehicleApproachingSubCauseCode.msg
    EnergyStorageType.msg
    EventHistory.msg
    EventPoint.msg
    ExteriorLights.msg
    HardShoulderStatus.msg
    HazardousLocation_AnimalOnTheRoadSubCauseCode.msg
    HazardousLocation_DangerousCurveSubCauseCode.msg
    HazardousLocation_ObstacleOnTheRoadSubCauseCode.msg
    HazardousLocation_SurfaceConditionSubCauseCode.msg
    HeadingConfidence.msg
    Heading.msg
    HeadingValue.msg
    HeightLonCarr.msg
    HumanPresenceOnTheRoadSubCauseCode.msg
    HumanProblemSubCauseCode.msg
    InformationQuality.msg
    ItineraryPath.msg
    ItsPduHeader.msg
    LanePosition.msg
    LateralAcceleration.msg
    LateralAccelerationValue.msg
    Latitude.msg
    LightBarSirenInUse.msg
    Longitude.msg
    LongitudinalAcceleration.msg
    LongitudinalAccelerationValue.msg
    NumberOfOccupants.msg
    OpeningDaysHours.msg
    PathDeltaTime.msg
    PathHistory.msg
    PathPoint.msg
    PerformanceClass.msg
    PhoneNumber.msg
    PosCentMass.msg
    PosConfidenceEllipse.msg
    PosFrontAx.msg
    PositioningSolutionType.msg
    PositionOfOccupants.msg
    PositionOfPillars.msg
    PosLonCarr.msg
    PosPillar.msg
    PostCrashSubCauseCode.msg
    ProtectedCommunicationZone.msg
    ProtectedCommunicationZonesRSU.msg
    ProtectedZoneID.msg
    ProtectedZoneRadius.msg
    ProtectedZoneType.msg
    PtActivationData.msg
    PtActivation.msg
    PtActivationType.msg
    ReferencePosition.msg
    RelevanceDistance.msg
    RelevanceTrafficDirection.msg
    RequestResponseIndication.msg
    RescueAndRecoveryWorkInProgressSubCauseCode.msg
    RestrictedTypes.msg
    RoadType.msg
    RoadworksSubCauseCode.msg
    SemiAxisLength.msg
    SequenceNumber.msg
    SignalViolationSubCauseCode.msg
    SlowVehicleSubCauseCode.msg
    SpecialTransportType.msg
    SpeedConfidence.msg
    SpeedLimit.msg
    Speed.msg
    SpeedValue.msg
    StationarySince.msg
    StationaryVehicleSubCauseCode.msg
    StationID.msg
    StationType.msg
    SteeringWheelAngleConfidence.msg
    SteeringWheelAngle.msg
    SteeringWheelAngleValue.msg
    SubCauseCodeType.msg
    Temperature.msg
    TimestampIts.msg
    Traces.msg
    TrafficConditionSubCauseCode.msg
    TrafficRule.msg
    TransmissionInterval.msg
    TurningRadius.msg
    ValidityDuration.msg
    VDS.msg
    VehicleBreakdownSubCauseCode.msg
    VehicleIdentification.msg
    VehicleLengthConfidenceIndication.msg
    VehicleLength.msg
    VehicleLengthValue.msg
    VehicleMass.msg
    VehicleRole.msg
    VehicleWidth.msg
    VerticalAcceleration.msg
    VerticalAccelerationValue.msg
    WheelBaseVehicle.msg
    WMInumber.msg
    WrongWayDrivingSubCauseCode.msg
    YawRateConfidence.msg
    YawRate.msg
    YawRateValue.msg
)

add_message_files(
  DIRECTORY
    msg/AddGrpC
  FILES
    BatteryStatus.msg
    ConnectionManeuverAssist_addGrpC.msg
    ConnectionTrajectory_addGrpC.msg
    EmissionType.msg
    ExceptionalCondition.msg
    IntersectionState_addGrpC.msg
    ItsStationPositionList.msg
    ItsStationPosition.msg
    LaneAttributes_addGrpC.msg
    MapData_addGrpC.msg
    MovementEvent_addGrpC.msg
    NodeAttributeSet_addGrpC.msg
    NodeLink.msg
    Node.msg
    Position3D_addGrpC.msg
    PrioritizationResponseList.msg
    PrioritizationResponse.msg
    PtvRequestType.msg
    RejectedReason.msg
    RequestorDescription_addGrpC.msg
    RestrictionUserType_addGrpC.msg
    SignalHeadLocationList.msg
    SignalHeadLocation.msg
    SignalStatusPackage_addGrpC.msg
    TimeReference.msg
)

add_message_files(
  DIRECTORY
    msg/DSRC
  FILES
    AdvisorySpeedList.msg
    AdvisorySpeed.msg
    AdvisorySpeedType.msg
    AllowedManeuvers.msg
    Angle.msg
    AntennaOffsetSet.msg
    ApproachID.msg
    BasicVehicleRole.msg
    ComputedLane.msg
    ConnectingLane.msg
    ConnectionManeuverAssist.msg
    Connection.msg
    ConnectsToList.msg
    DataParameters.msg
    DDateTime.msg
    DDay.msg
    DeltaAngle.msg
    DeltaTime.msg
    DescriptiveName.msg
    DHour.msg
    DMinute.msg
    DMonth.msg
    DOffset.msg
    DrivenLineOffsetLg.msg
    DrivenLineOffsetSm.msg
    DSecond.msg
    DSRCmsgID.msg
    DYear.msg
    ElevationConfidence.msg
    Elevation.msg
    EnabledLaneList.msg
    FuelType.msg
    FullPositionVector.msg
    GenericLane.msg
    GNSSstatus.msg
    IntersectionAccessPoint.msg
    IntersectionGeometryList.msg
    IntersectionGeometry.msg
    IntersectionID.msg
    IntersectionReferenceID.msg
    IntersectionStateList.msg
    IntersectionState.msg
    IntersectionStatusObject.msg
    LaneAttributes_Barrier.msg
    LaneAttributes_Bike.msg
    LaneAttributes_Crosswalk.msg
    LaneAttributes.msg
    LaneAttributes_Parking.msg
    LaneAttributes_Sidewalk.msg
    LaneAttributes_Striping.msg
    LaneAttributes_TrackedVehicle.msg
    LaneAttributes_Vehicle.msg
    LaneConnectionID.msg
    LaneDataAttributeList.msg
    LaneDataAttribute.msg
    LaneDirection.msg
    LaneID.msg
    LaneList.msg
    LaneSharing.msg
    LaneTypeAttributes.msg
    LaneWidth.msg
    LayerID.msg
    LayerType.msg
    ManeuverAssistList.msg
    MapData.msg
    MergeDivergeNodeAngle.msg
    MinuteOfTheYear.msg
    MovementEventList.msg
    MovementEvent.msg
    MovementList.msg
    MovementPhaseState.msg
    MovementState.msg
    MsgCount.msg
    NodeAttributeSetXY.msg
    NodeAttributeXYList.msg
    NodeAttributeXY.msg
    NodeListXY.msg
    Node_LLmD_64b.msg
    NodeOffsetPointXY.msg
    NodeSetXY.msg
    Node_XY_20b.msg
    Node_XY_22b.msg
    Node_XY_24b.msg
    Node_XY_26b.msg
    Node_XY_28b.msg
    Node_XY_32b.msg
    NodeXY.msg
    Offset_B09.msg
    Offset_B10.msg
    Offset_B11.msg
    Offset_B12.msg
    Offset_B13.msg
    Offset_B14.msg
    Offset_B16.msg
    OverlayLaneList.msg
    PedestrianBicycleDetect.msg
    Position3D.msg
    PositionalAccuracy.msg
    PositionConfidence.msg
    PositionConfidenceSet.msg
    PreemptPriorityList.msg
    PrioritizationResponseStatus.msg
    PriorityRequestType.msg
    RegionalExtension.msg
    RegionId.msg
    RegulatorySpeedLimit.msg
    RequestID.msg
    RequestImportanceLevel.msg
    RequestorDescription.msg
    RequestorPositionVector.msg
    RequestorType.msg
    RequestSubRole.msg
    RestrictionAppliesTo.msg
    RestrictionClassAssignment.msg
    RestrictionClassID.msg
    RestrictionClassList.msg
    RestrictionUserTypeList.msg
    RestrictionUserType.msg
    RoadLaneSetList.msg
    RoadRegulatorID.msg
    RoadSegmentID.msg
    RoadSegmentList.msg
    RoadSegment.msg
    RoadSegmentReferenceID.msg
    RoadwayCrownAngle.msg
    RTCMcorrections.msg
    RTCMheader.msg
    RTCMmessageList.msg
    RTCMmessage.msg
    RTCM_Revision.msg
    Scale_B12.msg
    SegmentAttributeXYList.msg
    SegmentAttributeXY.msg
    SemiMajorAxisAccuracy.msg
    SemiMajorAxisOrientation.msg
    SemiMinorAxisAccuracy.msg
    SignalControlZone.msg
    SignalGroupID.msg
    SignalRequesterInfo.msg
    SignalRequestList.msg
    SignalRequestMessage.msg
    SignalRequest.msg
    SignalRequestPackage.msg
    SignalStatusList.msg
    SignalStatusMessage.msg
    SignalStatus.msg
    SignalStatusPackageList.msg
    SignalStatusPackage.msg
    SPAT.msg
    SpeedAdvice.msg
    SpeedandHeadingandThrottleConfidence.msg
    SpeedLimitList.msg
    SpeedLimitType.msg
    TemporaryID.msg
    ThrottleConfidence.msg
    TimeChangeDetails.msg
    TimeConfidence.msg
    TimeIntervalConfidence.msg
    TimeMark.msg
    TransitVehicleOccupancy.msg
    TransitVehicleStatus.msg
    TransmissionAndSpeed.msg
    TransmissionState.msg
    VehicleHeight.msg
    VehicleID.msg
    VehicleType.msg
    Velocity.msg
    WaitOnStopline.msg
    ZoneLength.msg
)

add_message_files(
  DIRECTORY
    msg/ElectronicRegistrationIdentificationVehicleDataModule
  FILES
    EuVehicleCategoryCode.msg
    EuVehicleCategoryL.msg
    EuVehicleCategoryM.msg
    EuVehicleCategoryN.msg
    EuVehicleCategoryO.msg
    Iso3833VehicleType.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES etsi_its_msgs
  CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
# ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/etsi_its_msgs.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/etsi_its_msgs_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS
  scripts/asn1ToRosMsg.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_etsi_its_msgs.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
