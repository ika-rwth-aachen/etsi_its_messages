cmake_minimum_required(VERSION 3.5)
project(etsi_its_cam_msgs)

find_package(ros_environment REQUIRED QUIET)
set(ROS_VERSION $ENV{ROS_VERSION})
set(ROS_DISTRO $ENV{ROS_DISTRO})

# === ROS1 (CATKIN) ============================================================
if(${ROS_VERSION} EQUAL 1)

    find_package(catkin REQUIRED COMPONENTS
        message_generation
        std_msgs
    )

    ## Generate messages in the 'msg' folder
    add_message_files(DIRECTORY msg/CAM-PDU-Descriptions)
    add_message_files(DIRECTORY msg/ITS-Container)

    generate_messages(
        DEPENDENCIES
        std_msgs
    )

    catkin_package(
        CATKIN_DEPENDS
            message_runtime
            std_msgs
    )

# === ROS2 (AMENT) =============================================================
elseif(${ROS_VERSION} EQUAL 2)

    find_package(ament_cmake REQUIRED)
    find_package(rosidl_default_generators REQUIRED)
    find_package(std_msgs REQUIRED)

    set(msg_files
        # msg/CAM-PDU-Descriptions
        "msg/CAM-PDU-Descriptions/BasicContainer.msg"
        "msg/CAM-PDU-Descriptions/BasicVehicleContainerHighFrequency.msg"
        "msg/CAM-PDU-Descriptions/BasicVehicleContainerLowFrequency.msg"
        "msg/CAM-PDU-Descriptions/CAM.msg"
        "msg/CAM-PDU-Descriptions/CamParameters.msg"
        "msg/CAM-PDU-Descriptions/CoopAwareness.msg"
        "msg/CAM-PDU-Descriptions/DangerousGoodsContainer.msg"
        "msg/CAM-PDU-Descriptions/EmergencyContainer.msg"
        "msg/CAM-PDU-Descriptions/GenerationDeltaTime.msg"
        "msg/CAM-PDU-Descriptions/HighFrequencyContainer.msg"
        "msg/CAM-PDU-Descriptions/LowFrequencyContainer.msg"
        "msg/CAM-PDU-Descriptions/PublicTransportContainer.msg"
        "msg/CAM-PDU-Descriptions/RescueContainer.msg"
        "msg/CAM-PDU-Descriptions/RoadWorksContainerBasic.msg"
        "msg/CAM-PDU-Descriptions/RSUContainerHighFrequency.msg"
        "msg/CAM-PDU-Descriptions/SafetyCarContainer.msg"
        "msg/CAM-PDU-Descriptions/SpecialTransportContainer.msg"
        "msg/CAM-PDU-Descriptions/SpecialVehicleContainer.msg"
        # msg/ITS-Container
        "msg/ITS-Container/AccelerationConfidence.msg"
        "msg/ITS-Container/AccelerationControl.msg"
        "msg/ITS-Container/AltitudeConfidence.msg"
        "msg/ITS-Container/Altitude.msg"
        "msg/ITS-Container/AltitudeValue.msg"
        "msg/ITS-Container/CauseCode.msg"
        "msg/ITS-Container/CauseCodeType.msg"
        "msg/ITS-Container/CenDsrcTollingZoneID.msg"
        "msg/ITS-Container/CenDsrcTollingZone.msg"
        "msg/ITS-Container/ClosedLanes.msg"
        "msg/ITS-Container/CurvatureCalculationMode.msg"
        "msg/ITS-Container/CurvatureConfidence.msg"
        "msg/ITS-Container/Curvature.msg"
        "msg/ITS-Container/CurvatureValue.msg"
        "msg/ITS-Container/DangerousGoodsBasic.msg"
        "msg/ITS-Container/DeltaAltitude.msg"
        "msg/ITS-Container/DeltaLatitude.msg"
        "msg/ITS-Container/DeltaLongitude.msg"
        "msg/ITS-Container/DeltaReferencePosition.msg"
        "msg/ITS-Container/DriveDirection.msg"
        "msg/ITS-Container/DrivingLaneStatus.msg"
        "msg/ITS-Container/EmbarkationStatus.msg"
        "msg/ITS-Container/EmergencyPriority.msg"
        "msg/ITS-Container/ExteriorLights.msg"
        "msg/ITS-Container/HardShoulderStatus.msg"
        "msg/ITS-Container/HeadingConfidence.msg"
        "msg/ITS-Container/Heading.msg"
        "msg/ITS-Container/HeadingValue.msg"
        "msg/ITS-Container/ItsPduHeader.msg"
        "msg/ITS-Container/LanePosition.msg"
        "msg/ITS-Container/LateralAcceleration.msg"
        "msg/ITS-Container/LateralAccelerationValue.msg"
        "msg/ITS-Container/Latitude.msg"
        "msg/ITS-Container/LightBarSirenInUse.msg"
        "msg/ITS-Container/Longitude.msg"
        "msg/ITS-Container/LongitudinalAcceleration.msg"
        "msg/ITS-Container/LongitudinalAccelerationValue.msg"
        "msg/ITS-Container/PathDeltaTime.msg"
        "msg/ITS-Container/PathHistory.msg"
        "msg/ITS-Container/PathPoint.msg"
        "msg/ITS-Container/PerformanceClass.msg"
        "msg/ITS-Container/PosConfidenceEllipse.msg"
        "msg/ITS-Container/ProtectedCommunicationZone.msg"
        "msg/ITS-Container/ProtectedCommunicationZonesRSU.msg"
        "msg/ITS-Container/ProtectedZoneID.msg"
        "msg/ITS-Container/ProtectedZoneRadius.msg"
        "msg/ITS-Container/ProtectedZoneType.msg"
        "msg/ITS-Container/PtActivationData.msg"
        "msg/ITS-Container/PtActivation.msg"
        "msg/ITS-Container/PtActivationType.msg"
        "msg/ITS-Container/ReferencePosition.msg"
        "msg/ITS-Container/RoadworksSubCauseCode.msg"
        "msg/ITS-Container/SemiAxisLength.msg"
        "msg/ITS-Container/SpecialTransportType.msg"
        "msg/ITS-Container/SpeedConfidence.msg"
        "msg/ITS-Container/SpeedLimit.msg"
        "msg/ITS-Container/Speed.msg"
        "msg/ITS-Container/SpeedValue.msg"
        "msg/ITS-Container/StationID.msg"
        "msg/ITS-Container/StationType.msg"
        "msg/ITS-Container/SteeringWheelAngleConfidence.msg"
        "msg/ITS-Container/SteeringWheelAngle.msg"
        "msg/ITS-Container/SteeringWheelAngleValue.msg"
        "msg/ITS-Container/SubCauseCodeType.msg"
        "msg/ITS-Container/TimestampIts.msg"
        "msg/ITS-Container/TrafficRule.msg"
        "msg/ITS-Container/VehicleLengthConfidenceIndication.msg"
        "msg/ITS-Container/VehicleLength.msg"
        "msg/ITS-Container/VehicleLengthValue.msg"
        "msg/ITS-Container/VehicleRole.msg"
        "msg/ITS-Container/VehicleWidth.msg"
        "msg/ITS-Container/VerticalAcceleration.msg"
        "msg/ITS-Container/VerticalAccelerationValue.msg"
        "msg/ITS-Container/YawRateConfidence.msg"
        "msg/ITS-Container/YawRate.msg"
        "msg/ITS-Container/YawRateValue.msg"   
    )

    rosidl_generate_interfaces(${PROJECT_NAME}
        ${msg_files}
        DEPENDENCIES std_msgs
    )

    ament_export_dependencies(rosidl_default_runtime)
    
    ament_package()

endif()