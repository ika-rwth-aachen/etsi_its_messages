#include <ConversionNode.h>


namespace etsi_its_conversion {


ConversionNode::ConversionNode() : node_handle_(), private_node_handle_("~") {
  ROS_INFO("ConversionNode starting...");

  // setup publisher and subscriber
  cam_ros_pub_ = private_node_handle_.advertise<etsi_its_cam_msgs::CAM>("/CAM", 1);
  cam_asn1_pub_ = private_node_handle_.advertise<etsi_its_asn1_msgs::ASN1_udp>("/asn1/CAM", 1);
  cam_asn1_sub_ = private_node_handle_.subscribe("/asn1/CAM", 1, &ConversionNode::cam_asn1_callback, this);

  // create timer for publishing messages
  timer_ = node_handle_.createTimer(ros::Duration(1.0), &ConversionNode::generateDummyCAM, this);

  ros::spin();
}

void ConversionNode::generateDummyCAM(const ros::TimerEvent& event) {

  ROS_INFO("CAM Generator started.");

  // Create Dummy CAM ###############################################
  CAM_t etsiCAM;
  memset(&etsiCAM, 0, sizeof(CAM_t));
  ItsPduHeader_t header;
  memset(&header, 0, sizeof(ItsPduHeader_t));
  CoopAwareness_t	cam;
  memset(&cam, 0, sizeof(CoopAwareness_t));
  BasicContainer_t basicCont;
  memset(&basicCont, 0, sizeof(BasicContainer_t));
  HighFrequencyContainer_t highFreqCont;
  memset(&highFreqCont, 0, sizeof(HighFrequencyContainer_t));
  LowFrequencyContainer_t* lowFreqCont = (LowFrequencyContainer_t*)calloc(1, sizeof(LowFrequencyContainer_t));
  memset(lowFreqCont, 0, sizeof(LowFrequencyContainer_t));

  ROS_INFO("Structs init.");

  // Init ITS Header
  header.protocolVersion = 2;
  header.messageID = ItsPduHeader__messageID_cam;
  header.stationID = 20210217;
    etsiCAM.header = header;

  ROS_INFO("Header done.");

  // Basic Container
  basicCont.stationType = StationType_passengerCar;
  basicCont.referencePosition.latitude = Latitude_oneMicrodegreeNorth * 51.215169611787054 * 1000000.0; //in 0.1 mikro Degrees
  basicCont.referencePosition.longitude = Longitude_oneMicrodegreeEast * 6.776800439198526 * 1000000.0; //in 0.1 mikro Degrees
  basicCont.referencePosition.altitude.altitudeValue = AltitudeValue_unavailable; // in cm
  basicCont.referencePosition.altitude.altitudeConfidence = AltitudeConfidence_alt_200_00;
  basicCont.referencePosition.positionConfidenceEllipse.semiMajorConfidence = 1;
  basicCont.referencePosition.positionConfidenceEllipse.semiMinorConfidence = 1;
  basicCont.referencePosition.positionConfidenceEllipse.semiMajorOrientation = 0;
    cam.camParameters.basicContainer = basicCont;

  ROS_INFO("Basic Container done.");

  // High Frequency Container
  highFreqCont.present = HighFrequencyContainer_PR_basicVehicleContainerHighFrequency;
  highFreqCont.choice.basicVehicleContainerHighFrequency.heading.headingValue = HeadingValue_wgs84North;
  highFreqCont.choice.basicVehicleContainerHighFrequency.heading.headingConfidence = HeadingConfidence_equalOrWithinOneDegree;
  highFreqCont.choice.basicVehicleContainerHighFrequency.speed.speedValue = SpeedValue_oneCentimeterPerSec * 100;
  highFreqCont.choice.basicVehicleContainerHighFrequency.speed.speedConfidence = SpeedConfidence_equalOrWithinOneCentimeterPerSec;
  highFreqCont.choice.basicVehicleContainerHighFrequency.driveDirection = DriveDirection_forward;
  highFreqCont.choice.basicVehicleContainerHighFrequency.vehicleLength.vehicleLengthConfidenceIndication = VehicleLengthConfidenceIndication_noTrailerPresent;
  highFreqCont.choice.basicVehicleContainerHighFrequency.vehicleLength.vehicleLengthValue = VehicleLengthValue_tenCentimeters * 40;
  highFreqCont.choice.basicVehicleContainerHighFrequency.vehicleWidth = VehicleWidth_tenCentimeters * 20;
  highFreqCont.choice.basicVehicleContainerHighFrequency.longitudinalAcceleration.longitudinalAccelerationValue = 0;
  highFreqCont.choice.basicVehicleContainerHighFrequency.longitudinalAcceleration.longitudinalAccelerationConfidence = 1;
  highFreqCont.choice.basicVehicleContainerHighFrequency.curvature.curvatureValue = CurvatureValue_straight;
  highFreqCont.choice.basicVehicleContainerHighFrequency.curvature.curvatureConfidence = CurvatureConfidence_onePerMeter_0_1;
  highFreqCont.choice.basicVehicleContainerHighFrequency.curvatureCalculationMode = CurvatureCalculationMode_yawRateUsed;
  highFreqCont.choice.basicVehicleContainerHighFrequency.yawRate.yawRateValue = YawRateValue_straight;
  highFreqCont.choice.basicVehicleContainerHighFrequency.yawRate.yawRateConfidence = YawRateConfidence_degSec_000_01;
    cam.camParameters.highFrequencyContainer = highFreqCont;

  ROS_INFO("High Freq done.");

  // Low Frequency Container
  lowFreqCont->present = LowFrequencyContainer_PR_basicVehicleContainerLowFrequency;

  BIT_STRING_t extLights;
  extLights.size = 1;
  extLights.buf = (uint8_t*)calloc(extLights.size, sizeof(uint8_t));
  *(extLights.buf) = ExteriorLights_daytimeRunningLightsOn;
  extLights.bits_unused = 0;

  lowFreqCont->choice.basicVehicleContainerLowFrequency.exteriorLights = extLights;

  ROS_INFO("Low Freq Main done.");

  PathPoint_t* points = (PathPoint_t*)calloc(3, sizeof(PathPoint_t));

  for (int ppoint = 0; ppoint < 3; ppoint++)
  {
    PathPoint_t* pTemp = points+ppoint;
    
    pTemp->pathPosition.deltaLatitude = DeltaLatitude_oneMicrodegreeNorth * 10;
    pTemp->pathPosition.deltaLongitude = DeltaLongitude_oneMicrodegreeEast * 10;
    pTemp->pathPosition.deltaAltitude = DeltaAltitude_unavailable;
    //asn_set_add(&(lowFreqCont.choice.basicVehicleContainerLowFrequency.pathHistory.list), pTemp);
    ROS_INFO("Pt %d done.",ppoint);

    int ret = asn_sequence_add(&lowFreqCont->choice.basicVehicleContainerLowFrequency.pathHistory.list, pTemp);
    if (ret != 0) {
      ROS_WARN("&genericLane->nodeList.choice.nodes, node");
    } else {
      ROS_INFO("Add to seq");
    }

  }  

  ROS_INFO("Pts done.");
  
  // Init CAM content
  cam.generationDeltaTime = GenerationDeltaTime_oneMilliSec * 100;
  cam.camParameters.lowFrequencyContainer = NULL;
  cam.camParameters.specialVehicleContainer = NULL;
  cam.camParameters.lowFrequencyContainer = lowFreqCont;
  etsiCAM.cam = cam;

  ROS_INFO("Full Msg done.");

  // ASN.1-Coding
  char errbuf[1024];
  size_t errlen = sizeof(errbuf);
  int checkRet = asn_check_constraints(&asn_DEF_CAM, &etsiCAM, errbuf, &errlen);
  if (checkRet)
  {
      ROS_ERROR("Error: %s", errbuf);
      return;
  }
  asn_encode_to_new_buffer_result_t res = asn_encode_to_new_buffer(0, ATS_UNALIGNED_BASIC_PER, &asn_DEF_CAM, &etsiCAM);
  if (res.result.encoded == -1)
  {
      ROS_ERROR("Err: %s", res.result.failed_type->xml_tag);
  }
  ROS_INFO("CAM-Length: %i", (int)res.result.encoded);

  asn_fprint(stdout, &asn_DEF_CAM, &etsiCAM);

  etsi_its_asn1_msgs::ASN1_udp cam_asn1;
  cam_asn1.header.stamp = ros::Time::now();
  cam_asn1.header.frame_id = "base_link";
  cam_asn1.data = std::vector<uint8_t>((char *)res.buffer, (char *)res.buffer + (int)res.result.encoded);
  cam_asn1_pub_.publish(cam_asn1);
}

void ConversionNode::cam_asn1_callback(etsi_its_asn1_msgs::ASN1_udp msg)
{
  CAM_t *camPdu = 0;
  asn_dec_rval_t decodeRet = asn_decode(0, ATS_UNALIGNED_BASIC_PER, &asn_DEF_CAM, (void **)&camPdu, &msg.data[0], msg.data.size());
  asn_fprint(stdout, &asn_DEF_CAM, camPdu);
  auto camROS = etsi_its_cam_conversion::convert_CAMtoRos(*camPdu);
  cam_ros_pub_.publish(camROS);
}


}  // end of namespace


int main(int argc, char **argv)
{
  ros::init(argc, argv, "ConversionNode");

  etsi_its_conversion::ConversionNode node;

  return 0;
}

