/**
 * @file
 * @brief Access functions for etsi_its_cam_msgs
 */

#pragma once

namespace etsi_its_cam_msgs {

namespace access_functions {

  void setItsPduHeader(CAM& cam, int station_id, int protocol_version = 1) {
    cam.header.messageID = ItsPduHeader::MESSAGE_I_D_CAM;
    cam.header.stationID.value = station_id;
    if(protocol_version>=0 && protocol_version<=255) cam.header.protocolVersion = protocol_version;
    else std::invalid_argument("ProtocolVersion is out of Range (0...255)!");
  }

} // namespace access_functions

} // namespace etsi_its_cam_msgs
