cmake_minimum_required(VERSION 3.5)
project(etsi_its_rtcmem_ts_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  "msg/AntennaOffsetSet.msg"
  "msg/DDateTime.msg"
  "msg/DDay.msg"
  "msg/DHour.msg"
  "msg/DMinute.msg"
  "msg/DMonth.msg"
  "msg/DOffset.msg"
  "msg/DSecond.msg"
  "msg/DYear.msg"
  "msg/Elevation.msg"
  "msg/ElevationConfidence.msg"
  "msg/FullPositionVector.msg"
  "msg/GNSSstatus.msg"
  "msg/HeadingConfidenceDSRC.msg"
  "msg/HeadingDSRC.msg"
  "msg/ItsPduHeader.msg"
  "msg/Latitude.msg"
  "msg/Longitude.msg"
  "msg/MinuteOfTheYear.msg"
  "msg/MsgCount.msg"
  "msg/OffsetB09.msg"
  "msg/OffsetB10.msg"
  "msg/OffsetB12.msg"
  "msg/PositionConfidence.msg"
  "msg/PositionConfidenceSet.msg"
  "msg/PositionalAccuracy.msg"
  "msg/RTCMEM.msg"
  "msg/RTCMRevision.msg"
  "msg/RTCMcorrections.msg"
  "msg/RTCMheader.msg"
  "msg/RTCMmessage.msg"
  "msg/RTCMmessageList.msg"
  "msg/SemiMajorAxisAccuracy.msg"
  "msg/SemiMajorAxisOrientation.msg"
  "msg/SemiMinorAxisAccuracy.msg"
  "msg/SpeedConfidenceDSRC.msg"
  "msg/SpeedandHeadingandThrottleConfidence.msg"
  "msg/StationID.msg"
  "msg/ThrottleConfidence.msg"
  "msg/TimeConfidence.msg"
  "msg/TransmissionAndSpeed.msg"
  "msg/TransmissionState.msg"
  "msg/Velocity.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
