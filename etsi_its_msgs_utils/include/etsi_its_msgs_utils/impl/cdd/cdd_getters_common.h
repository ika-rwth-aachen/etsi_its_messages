/*
=============================================================================
MIT License

Copyright (c) 2023-2025 Institute for Automotive Engineering (ika), RWTH Aachen University

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
=============================================================================
*/

/**
 * @file impl/cdd/cdd_getters_common.h
 * @brief Common getter functions for the ETSI ITS Common Data Dictionary (CDD) v1.3.1 and v2.1.1
 */

#ifndef ETSI_ITS_MSGS_UTILS_IMPL_CDD_CDD_GETTERS_COMMON_H
#define ETSI_ITS_MSGS_UTILS_IMPL_CDD_CDD_GETTERS_COMMON_H

#include <array>
#include <cmath>
#include <cstdint>
#include <GeographicLib/UTMUPS.hpp>

/**
* @brief Get the StationID of ItsPduHeader
*
* @param header ItsPduHeader to get the StationID value from
* @return stationID value
*/
inline uint32_t getStationID(const ItsPduHeader& header) { return header.station_id.value; }

/**
 * @brief Get the Latitude value
 *
 * @param latitude to get the Latitude value from
 * @return Latitude value in degree as decimal number
 */
inline double getLatitude(const Latitude& latitude) { return ((double)latitude.value) * 1e-7; }

/**
 * @brief Get the Longitude value
 *
 * @param longitude to get the Longitude value from
 * @return Longitude value in degree as decimal number
 */
inline double getLongitude(const Longitude& longitude) { return ((double)longitude.value) * 1e-7; }

/**
 * @brief Get the Altitude value
 *
 * @param altitude to get the Altitude value from
 * @return Altitude value (above the reference ellipsoid surface) in meter as decimal number
 */
inline double getAltitude(const Altitude& altitude) { return ((double)altitude.altitude_value.value) * 1e-2; }

/**
 * @brief Get the vehicle speed
 *
 * @param speed to get the speed value from
 * @return speed value in m/s as decimal number
 */
inline double getSpeed(const Speed& speed) { return ((double)speed.speed_value.value) * 1e-2; }

/**
 * @brief Get the Speed Confidence 
 * 
 * @param speed to get the SpeedConfidence from
 * @return double speed standard deviation in m/s as decimal number
 */
inline double getSpeedConfidence(const Speed& speed) {
  return ((double)speed.speed_confidence.value) / etsi_its_msgs::ONE_D_GAUSSIAN_FACTOR * 1e-2;
}

/**
 * @brief Get the UTM Position defined by the given ReferencePosition
 *
 * The position is transformed into UTM by using GeographicLib::UTMUPS
 * The altitude value is directly used as z-Coordinate
 *
 * @param[in] reference_position ReferencePosition or ReferencePositionWithConfidence to get the UTM Position from
 * @param[out] zone the UTM zone (zero means UPS)
 * @param[out] northp hemisphere (true means north, false means south)
 * @return gm::PointStamped geometry_msgs::PointStamped of the given position
 */
template <typename T>
inline gm::PointStamped getUTMPosition(const T& reference_position, int& zone, bool& northp) {
  gm::PointStamped utm_point;
  double latitude = getLatitude(reference_position.latitude);
  double longitude = getLongitude(reference_position.longitude);
  utm_point.point.z = getAltitude(reference_position.altitude);
  try {
    GeographicLib::UTMUPS::Forward(latitude, longitude, zone, northp, utm_point.point.x, utm_point.point.y);
    std::string hemisphere;
    if (northp) {
      hemisphere = "N";
    } else {
      hemisphere = "S";
    }
    utm_point.header.frame_id = "utm_" + std::to_string(zone) + hemisphere;
  } catch (GeographicLib::GeographicErr& e) {
    throw std::invalid_argument(e.what());
  }
  return utm_point;
}

/**
 * @brief Get the Heading value
 *
 * 0.0° equals WGS84 North, 90.0° equals WGS84 East, 180.0° equals WGS84 South and 270.0° equals WGS84 West
 *
 * @param heading to get the Heading value from
 * @return Heading value in degree as decimal number
 */
template <typename Heading>
inline double getHeadingCDD(const Heading& heading) { return ((double)heading.heading_value.value) * 1e-1; }

/**
 * @brief Get the Heading value
 *
 * 0.0° equals WGS84 North, 90.0° equals WGS84 East, 180.0° equals WGS84 South and 270.0° equals WGS84 West
 *
 * @param heading to get the Heading standard deviation from
 * @return Heading standard deviation in degree as decimal number
 */
template <typename Heading>
inline double getHeadingConfidenceCDD(const Heading& heading) { return ((double)heading.heading_confidence.value) * 1e-1 / etsi_its_msgs::ONE_D_GAUSSIAN_FACTOR; }

/**
 * @brief Get the Semi Axis object
 * 
 * @param semi_axis_length The SemiAxisLength object to get the semi axis from
 * @return double the semi axis length in meters
 */
template <typename SemiAxisLength>
inline double getSemiAxis(const SemiAxisLength& semi_axis_length) {
  return ((double)semi_axis_length.value) * 1e-2 / etsi_its_msgs::OneCentimeterHelper<SemiAxisLength>::value;
}

/**
 * @brief Extract major axis length, minor axis length and orientation from the given position confidence ellipse
 * 
 * @param position_confidence_ellipse The position confidence ellipse to extract the values from
 * @return std::tuple<double, double, double> major axis length in meters, minor axis length in meters, and orientation in degrees
 */
template <typename PosConfidenceEllipse>
inline std::tuple<double, double, double> getPosConfidenceEllipse(const PosConfidenceEllipse& position_confidence_ellipse) {
  return {
    getSemiAxis(position_confidence_ellipse.semi_major_confidence),
    getSemiAxis(position_confidence_ellipse.semi_minor_confidence),
    position_confidence_ellipse.semi_major_orientation.value * 1e-1
  };
}

/**
 * @brief Convert the confidence ellipse to a covariance matrix
 * 
 * Note that the major_orientation is given in degrees, while the object_heading is given in radians!
 * 
 * @param semi_major Semi major axis length in meters
 * @param semi_minor Semi minor axis length in meters
 * @param major_orientation Orientation of the major axis in degrees, relative to WGS84
 * @param object_heading object heading in radians, relative to WGS84
 * @return std::array<double, 4> The covariance matrix in vehicle coordinates (x = longitudinal, y = lateral)
 */
inline std::array<double, 4> CovMatrixFromConfidenceEllipse(double semi_major, double semi_minor, double major_orientation, const double object_heading) {
  std::array<double, 4> covariance_matrix;
  double semi_major_squared = semi_major * semi_major / (etsi_its_msgs::TWO_D_GAUSSIAN_FACTOR * etsi_its_msgs::TWO_D_GAUSSIAN_FACTOR);
  double semi_minor_squared = semi_minor * semi_minor / (etsi_its_msgs::TWO_D_GAUSSIAN_FACTOR * etsi_its_msgs::TWO_D_GAUSSIAN_FACTOR);
  double major_orientation_rad = major_orientation * M_PI / 180;
  double object_heading_rad = object_heading;
  
  double angle_to_object = object_heading_rad - major_orientation_rad;

  double cos_angle = std::cos(angle_to_object);
  double sin_angle = std::sin(angle_to_object);
  covariance_matrix[0] = semi_major_squared * cos_angle * cos_angle + semi_minor_squared * sin_angle * sin_angle;
  covariance_matrix[1] = (semi_major_squared - semi_minor_squared) * cos_angle * sin_angle;
  covariance_matrix[2] = covariance_matrix[1];
  covariance_matrix[3] = semi_major_squared * sin_angle * sin_angle + semi_minor_squared * cos_angle * cos_angle;

  return covariance_matrix;
}

/**
 * @brief Convert the confidence ellipse to a covariance matrix
 * 
 * Note that the major_orientation is given in degrees, while the object_heading is given in radians!
 * 
 * @param semi_major Semi major axis length in meters
 * @param semi_minor Semi minor axis length in meters
 * @param major_orientation Orientation of the major axis in degrees, relative to WGS84
 * @return std::array<double, 4> The covariance matrix in WGS coordinates (x = North, y = East)
 */
inline std::array<double, 4> WGSCovMatrixFromConfidenceEllipse(double semi_major, double semi_minor, double major_orientation) {
  // The WGS covariance matrix is the same as in vehicle coordinates, if it would have a heading of 0.0
  return CovMatrixFromConfidenceEllipse(semi_major, semi_minor, major_orientation, 0.0);
}

/**
 * @brief Get the covariance matrix of the position confidence ellipse
 * 
 * @param position_confidence_ellipse The position confidence ellipse to get the covariance matrix from
 * @param object_heading The object heading in radians
 * @return std::array<double, 4> The covariance matrix of the position confidence ellipse in vehicle coordinates (x = longitudinal, y = lateral)
 */
template <typename PosConfidenceEllipse>
inline std::array<double, 4> getPosConfidenceEllipse(const PosConfidenceEllipse& position_confidence_ellipse, const double object_heading){
  auto [semi_major, semi_minor, major_orientation] = getPosConfidenceEllipse(position_confidence_ellipse);
  return CovMatrixFromConfidenceEllipse(semi_major, semi_minor, major_orientation, object_heading);
}

/**
 * @brief Get the covariance matrix of the position confidence ellipse
 * 
 * @param position_confidence_ellipse The position confidence ellipse to get the covariance matrix from
 * @param object_heading The object heading in radians
 * @return std::array<double, 4> The covariance matrix of the position confidence ellipse in WGS coordinates (x = North, y = East)
 */
template <typename PosConfidenceEllipse>
inline std::array<double, 4> getWGSPosConfidenceEllipse(const PosConfidenceEllipse& position_confidence_ellipse){
  auto [semi_major, semi_minor, major_orientation] = getPosConfidenceEllipse(position_confidence_ellipse);
  return WGSCovMatrixFromConfidenceEllipse(semi_major, semi_minor, major_orientation);
}

#endif  // ETSI_ITS_MSGS_UTILS_IMPL_CDD_CDD_GETTERS_COMMON_H