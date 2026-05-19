// SPDX-License-Identifier: MIT
// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University

/**
 * @file impl/cam/cam_ts_getters.h
 * @brief Getter functions for the ETSI ITS CAM (TS)
 */

#pragma once

namespace etsi_its_cam_ts_msgs::access {
#include <etsi_its_msgs_utils/impl/cdd/cdd_v2-1-1_getters.h>

#include <etsi_its_msgs_utils/impl/cam/cam_getters_common.h>

/**
 * @brief Get the confidence ellipse of the reference position as Covariance matrix
 *
 * The covariance matrix will have the entries cov_xx, cov_xy, cov_yx, cov_yy
 * where x is the longitudinal axis and y is the lateral axis of the vehicle.
 *
 * @param cam The CAM message to get the reference position from
 * @return const std::array<double, 4> the covariance matrix, as specified above
 */
inline const std::array<double, 4> getRefPosConfidence(const CAM& cam) {
  double object_heading = getHeading(cam) * M_PI / 180.0;
  return getPositionConfidenceEllipse(cam.cam.cam_parameters.basic_container.reference_position.position_confidence_ellipse, object_heading);
}

/**
 * @brief Get the confidence ellipse of the reference position as Covariance matrix
 *
 * The covariance matrix will have the entries cov_xx, cov_xy, cov_yx, cov_yy
 * where x is WGS84 North and y is East
 *
 * @param cam The CAM message to get the reference position from
 * @return const std::array<double, 4> the covariance matrix, as specified above
 */
inline const std::array<double, 4> getWGSRefPosConfidence(const CAM& cam) {
  return getWGSPositionConfidenceEllipse(cam.cam.cam_parameters.basic_container.reference_position.position_confidence_ellipse);
}

}  // namespace etsi_its_cam_ts_msgs::access
