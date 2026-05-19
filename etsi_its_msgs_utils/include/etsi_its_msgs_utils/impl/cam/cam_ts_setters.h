// SPDX-License-Identifier: MIT
// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University

/**
 * @file impl/cam/cam_ts_setters.h
 * @brief Setter functions for the ETSI ITS CAM (TS)
 */

#pragma once

namespace etsi_its_cam_ts_msgs::access {

#include <etsi_its_msgs_utils/impl/cdd/cdd_v2-1-1_setters.h>

#include <etsi_its_msgs_utils/impl/cam/cam_setters_common.h>

/**
 * @brief Set the ItsPduHeader-object for a CAM
 *
 * @param cam CAM-Message to set the ItsPduHeader
 * @param station_id
 * @param protocol_version
 */
inline void setItsPduHeader(CAM& cam, const uint32_t station_id, const uint8_t protocol_version = 0) {
  setItsPduHeader(cam.header, MessageId::CAM, station_id, protocol_version);
}

/**
 * @brief Set the confidence of the reference position
 *
 * @param cam CAM-Message to set the confidence
 * @param covariance_matrix The four values of the covariance matrix in the order: cov_xx, cov_xy, cov_yx, cov_yy
 *                          The matrix has to be SPD, otherwise a std::invalid_argument exception is thrown.
 *                          Its coordinate system is aligned with the object (x = longitudinal, y = lateral)
 * @param object_heading heading of the object in rad, with respect to WGS84
 */
inline void setRefPosConfidence(CAM& cam, const std::array<double, 4>& covariance_matrix, const double object_heading) {
  // First ensure, that the object has the correct heading by setting its value
  double orientation = object_heading * 180 / M_PI; // Convert to degrees
  // Normalize to [0, 360)
  orientation = std::fmod(orientation + 360, 360);
  while (orientation < 0) {
    orientation += 360;
  }
  while (orientation >= 360) {
    orientation -= 360;
  }
  setHeading(cam, orientation);
  setPositionConfidenceEllipse(cam.cam.cam_parameters.basic_container.reference_position.position_confidence_ellipse,
                          covariance_matrix, object_heading);
}

/**
 * @brief Set the confidence of the reference position
 *
 * @param cam CAM-Message to set the confidence
 * @param covariance_matrix The four values of the covariance matrix in the order: cov_xx, cov_xy, cov_yx, cov_yy
 *                          The matrix has to be SPD, otherwise a std::invalid_argument exception is thrown.
 *                          Its coordinate system is WGS84 (x = North, y = East)
 */
inline void setWGSRefPosConfidence(CAM& cam, const std::array<double, 4>& covariance_matrix) {
  setWGSPositionConfidenceEllipse(cam.cam.cam_parameters.basic_container.reference_position.position_confidence_ellipse,
                                  covariance_matrix);
}


}  // namespace etsi_its_cam_ts_msgs::access
