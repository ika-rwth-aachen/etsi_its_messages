/**
 * @file
 * @brief Access functions for etsi_its_cam_msgs
 */

#pragma once

namespace etsi_its_cam_msgs {

namespace access_functions {

  /**
   * @brief Set the Its Pdu Header object
   * 
   * @param header ItsPduHeader to be set 
   * @param station_id 
   * @param protocol_version 
   */
  void setItsPduHeader(ItsPduHeader& header, int station_id, int protocol_version = 1) {
    header.messageID = ItsPduHeader::MESSAGE_I_D_CAM;
    header.stationID.value = station_id;
    if(protocol_version>=0 && protocol_version<=255) header.protocolVersion = protocol_version;
    else std::invalid_argument("ProtocolVersion is out of Range (0...255)!");
  }
  
  /**
   * @brief Set the ItsPduHeader-object
   * 
   * @param cam CAM-Message to set the ItsPduHeader
   * @param station_id
   * @param protocol_version 
   */
  void setItsPduHeader(CAM& cam, int station_id, int protocol_version = 1) {
    setItsPduHeader(cam.header, station_id, protocol_version);
  }

  void setReferencePosition(ReferencePosition& ref_pos, int latitude, int longitude, int altitude, PosConfidenceEllipse pos_conf_ellipse) {
    if(latitude>=-900000000 && latitude<=900000001) ref_pos.latitude.value = latitude;
    else ref_pos.latitude.value = Latitude::UNAVAILABLE;
    if(longitude>=-1800000000 && longitude<=1800000001) ref_pos.longitude.value = longitude;
    else ref_pos.longitude.value = Longitude::UNAVAILABLE;
    setAltitude(ref_pos.altitude, altitude)
  }

} // namespace access_functions

} // namespace etsi_its_cam_msgs
