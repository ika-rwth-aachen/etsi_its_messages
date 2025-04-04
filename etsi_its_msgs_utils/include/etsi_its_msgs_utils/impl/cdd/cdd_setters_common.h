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
 * @file impl/cdd/cdd_setters_common.h
 * @brief Common setter functions for the ETSI ITS Common Data Dictionary (CDD) v1.3.1 and v2.1.1
 */

#ifndef ETSI_ITS_MSGS_UTILS_IMPL_CDD_CDD_SETTERS_COMMON_H
#define ETSI_ITS_MSGS_UTILS_IMPL_CDD_CDD_SETTERS_COMMON_H

#include <etsi_its_msgs_utils/impl/checks.h>
#include <etsi_its_msgs_utils/impl/constants.h>
#include <GeographicLib/UTMUPS.hpp>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <type_traits>

/**
 * @brief Set the TimestampITS object
 *
 * @param[in] timestamp_its TimestampITS object to set the timestamp
 * @param[in] unix_nanosecs Unix-Nanoseconds to set the timestamp for
 * @param[in] n_leap_seconds Number of leap-seconds since 2004. (Defaults to the todays number of leap seconds since 2004.)
 * @param[in] epoch_offset Unix-Timestamp in seconds for the 01.01.2004 at 00:00:00
 */
inline void setTimestampITS(
    TimestampIts& timestamp_its, const uint64_t unix_nanosecs,
    const uint16_t n_leap_seconds = etsi_its_msgs::LEAP_SECOND_INSERTIONS_SINCE_2004.rbegin()->second) {
  uint64_t t_its = unix_nanosecs * 1e-6 + (uint64_t)(n_leap_seconds * 1e3) - etsi_its_msgs::UNIX_SECONDS_2004 * 1e3;
  throwIfOutOfRange(t_its, TimestampIts::MIN, TimestampIts::MAX, "TimestampIts");
  timestamp_its.value = t_its;
}

/**
 * @brief Set the Latitude object
 *
 * @param latitude object to set
 * @param deg Latitude value in degree as decimal number
 */
inline void setLatitude(Latitude& latitude, const double deg) {
  int64_t angle_in_10_micro_degree = (int64_t)std::round(deg * 1e7);
  throwIfOutOfRange(angle_in_10_micro_degree, Latitude::MIN, Latitude::MAX, "Latitude");
  latitude.value = angle_in_10_micro_degree;
}

/**
 * @brief Set the Longitude object
 *
 * @param longitude object to set
 * @param deg Longitude value in degree as decimal number
 */
inline void setLongitude(Longitude& longitude, const double deg) {
  int64_t angle_in_10_micro_degree = (int64_t)std::round(deg * 1e7);
  throwIfOutOfRange(angle_in_10_micro_degree, Longitude::MIN, Longitude::MAX, "Longitude");
  longitude.value = angle_in_10_micro_degree;
}

/**
 * @brief Set the AltitudeValue object
 *
 * @param altitude object to set
 * @param value AltitudeValue value (above the reference ellipsoid surface) in meter as decimal number
 */
inline void setAltitudeValue(AltitudeValue& altitude, const double value) {
  int64_t alt_in_cm = (int64_t)std::round(value * 1e2);
  if (alt_in_cm >= AltitudeValue::MIN && alt_in_cm <= AltitudeValue::MAX) {
    altitude.value = alt_in_cm;
  } else if (alt_in_cm < AltitudeValue::MIN) {
    altitude.value = AltitudeValue::MIN;
  } else if (alt_in_cm > AltitudeValue::MAX) {
    altitude.value = AltitudeValue::MAX;
  }
}

/**
 * @brief Set the Altitude object
 *
 * AltitudeConfidence is set to UNAVAILABLE
 *
 * @param altitude object to set
 * @param value Altitude value (above the reference ellipsoid surface) in meter as decimal number
 */
inline void setAltitude(Altitude& altitude, const double value) {
  altitude.altitude_confidence.value = AltitudeConfidence::UNAVAILABLE;
  setAltitudeValue(altitude.altitude_value, value);
}

/**
 * @brief Set the SpeedValue object
 *
 * @param speed object to set
 * @param value SpeedValue in m/s as decimal number
 */
inline void setSpeedValue(SpeedValue& speed, const double value) {
  uint16_t speed_val = (uint16_t)std::round(value * 1e2);
  throwIfOutOfRange(speed_val, SpeedValue::MIN, SpeedValue::MAX, "SpeedValue");
  speed.value = speed_val;
}

/**
 * @brief Set the Speed object
 *
 * SpeedConfidence is set to UNAVAILABLE
 *
 * @param speed object to set
 * @param value  Speed in in m/s as decimal number
 */
inline void setSpeed(Speed& speed, const double value) {
  speed.speed_confidence.value = SpeedConfidence::UNAVAILABLE;
  setSpeedValue(speed.speed_value, value);
}

/**
 * @brief Sets the reference position in the given ReferencePostion object.
 * 
 * This function sets the latitude, longitude, and altitude of the reference position.
 * If the altitude is not provided, it is set to AltitudeValue::UNAVAILABLE.
 * 
 * @param ref_position ReferencePostion or ReferencePositionWithConfidence object to set the reference position in.
 * @param latitude The latitude value position in degree as decimal number.
 * @param longitude The longitude value in degree as decimal number.
 * @param altitude The altitude value (above the reference ellipsoid surface) in meter as decimal number (optional).
 */
template <typename T>
inline void setReferencePosition(T& ref_position, const double latitude, const double longitude,
                                 const double altitude = AltitudeValue::UNAVAILABLE) {
  setLatitude(ref_position.latitude, latitude);
  setLongitude(ref_position.longitude, longitude);
  if (altitude != AltitudeValue::UNAVAILABLE) {
    setAltitude(ref_position.altitude, altitude);
  } else {
    ref_position.altitude.altitude_value.value = AltitudeValue::UNAVAILABLE;
    ref_position.altitude.altitude_confidence.value = AltitudeConfidence::UNAVAILABLE;
  }
  // TODO: set confidence values
}

/**
 * @brief Set the ReferencePosition from a given UTM-Position
 *
 * The position is transformed to latitude and longitude by using GeographicLib::UTMUPS
 * The z-Coordinate is directly used as altitude value
 * The frame_id of the given utm_position must be set to 'utm_<zone><N/S>'
 *
 * @param[out] reference_position ReferencePostion or ReferencePositionWithConfidence to set
 * @param[in] utm_position geometry_msgs::PointStamped describing the given utm position
 * @param[in] zone the UTM zone (zero means UPS) of the given position
 * @param[in] northp hemisphere (true means north, false means south)
 */
template <typename T>
inline void setFromUTMPosition(T& reference_position, const gm::PointStamped& utm_position, const int zone,
                               const bool northp) {
  std::string required_frame_prefix = "utm_";
  if (utm_position.header.frame_id.rfind(required_frame_prefix, 0) != 0) {
    throw std::invalid_argument("Frame-ID of UTM Position '" + utm_position.header.frame_id +
                                "' does not start with required prefix '" + required_frame_prefix + "'!");
  }
  double latitude, longitude;
  try {
    GeographicLib::UTMUPS::Reverse(zone, northp, utm_position.point.x, utm_position.point.y, latitude, longitude);
  } catch (GeographicLib::GeographicErr& e) {
    throw std::invalid_argument(e.what());
  }
  setReferencePosition(reference_position, latitude, longitude, utm_position.point.z);
}

/**
 * @brief Set the HeadingValue object
 *
 * 0.0° equals WGS84 North, 90.0° equals WGS84 East, 180.0° equals WGS84 South and 270.0° equals WGS84 West
 *
 * @param heading object to set
 * @param value Heading value in degree as decimal number
 */
template <typename HeadingValue>
inline void setHeadingValue(HeadingValue& heading, const double value) {
  int64_t deg = (int64_t)std::round(value * 1e1);
  throwIfOutOfRange(deg, HeadingValue::MIN, HeadingValue::MAX, "HeadingValue");
  heading.value = deg;
}

/**
 * @brief Set the Heading object
 *
 * 0.0° equals WGS84 North, 90.0° equals WGS84 East, 180.0° equals WGS84 South and 270.0° equals WGS84 West
 * HeadingConfidence is set to UNAVAILABLE
 *
 * @param heading object to set
 * @param value Heading value in degree as decimal number
 */
template <typename Heading, typename HeadingConfidence = decltype(Heading::heading_confidence)>
void setHeadingInternal(Heading& heading, const double value) {
  heading.heading_confidence.value = HeadingConfidence::UNAVAILABLE;
  setHeadingValue(heading.heading_value, value);
}

template <typename SemiAxisLength, typename = std::void_t<>>
struct OneCentimeterHelper {
    static constexpr int value = 1; // Default value
};

template <typename SemiAxisLength>
struct OneCentimeterHelper<SemiAxisLength, std::void_t<decltype(SemiAxisLength::ONE_CENTIMETER)>> {
    static constexpr int value = SemiAxisLength::ONE_CENTIMETER;
};

// See https://godbolt.org/z/Eceavfo99
template <typename SemiAxisLength>
inline void setSemiAxis(SemiAxisLength& semi_axis_length, const double length) {
  double semi_axis_length_val = std::round(length * OneCentimeterHelper<SemiAxisLength>::value * 1e2);
  if(semi_axis_length_val < 1) {
    semi_axis_length_val = SemiAxisLength::UNAVAILABLE;
  } else if(semi_axis_length_val >= SemiAxisLength::OUT_OF_RANGE) {
    semi_axis_length_val = SemiAxisLength::OUT_OF_RANGE;
  }  
  semi_axis_length.value = static_cast<uint16_t>(semi_axis_length_val);
}

/**
 * @brief Set the Pos Confidence Ellipse object
 * 
 * @param[out] position_confidence_ellipse The PosConfidenceEllipse to set
 * @param[in] semi_major_axis length of the semi-major axis in meters
 * @param[in] semi_minor_axis length of the semi-minor axis in meters
 * @param[in] orientation of the semi-major axis in degrees, with respect to WGS84
 */
template <typename PosConfidenceEllipse>
inline void setPosConfidenceEllipse(PosConfidenceEllipse& position_confidence_ellipse, const double semi_major_axis,
  const double semi_minor_axis, const double orientation) {
  setSemiAxis(position_confidence_ellipse.semi_major_confidence, semi_major_axis);
  setSemiAxis(position_confidence_ellipse.semi_minor_confidence, semi_minor_axis);
  setHeadingValue(position_confidence_ellipse.semi_major_orientation, orientation);
}

/**
 * @brief Gets the values needed to set a confidence ellipse from a covariance matrix.
 * 
 * @param covariance_matrix The four values of the covariance matrix in the order: cov_xx, cov_xy, cov_yx, cov_yy
 *                          The matrix has to be SPD, otherwise a std::invalid_argument exception is thrown.
 *                          Its coordinate system is aligned with the object (x = longitudinal, y = lateral)
 * @param object_heading The heading of the object in rad, with respect to WGS84
 * @return std::tuple<double, double, double> semi_major_axis [m], semi_minor_axis [m], orientation [deg], with respect to WGS84
 */
inline std::tuple<double, double, double> confidenceEllipseFromCovMatrix(const std::array<double, 4>& covariance_matrix, const double object_heading) {
  
  if(std::abs(covariance_matrix[1] - covariance_matrix[2]) > 1e-6) {
    throw std::invalid_argument("Covariance matrix is not symmetric");
  }
  double trace = covariance_matrix[0] + covariance_matrix[3];
  double determinant = covariance_matrix[0] * covariance_matrix[3] - covariance_matrix[1] * covariance_matrix[1];
  if (determinant <= 0 || covariance_matrix[0] <= 0) {
    // https://sites.math.northwestern.edu/~clark/285/2006-07/handouts/pos-def.pdf:
    // Therefore, a necessary and sufficient condition for the quadratic form of a symmetric 2 × 2 matrix
    // to be positive definite is for det(A) > 0 and a > 0
    throw std::invalid_argument("Covariance matrix is not positive definite");
  }
  double eigenvalue1 = trace / 2 + std::sqrt(trace * trace / 4 - determinant);
  double eigenvalue2 = trace / 2 - std::sqrt(trace * trace / 4 - determinant);
  double semi_major_axis = std::sqrt(eigenvalue1) * etsi_its_msgs::TWO_D_GAUSSIAN_FACTOR;
  double semi_minor_axis = std::sqrt(eigenvalue2) * etsi_its_msgs::TWO_D_GAUSSIAN_FACTOR;
  // object_heading - orientation of the ellipse, as WGS84 has positive angles to the right
  double orientation = object_heading - 0.5 * std::atan2(2 * covariance_matrix[1], covariance_matrix[0] - covariance_matrix[3]);
  std::cerr << "Covariance matrix: " << std::endl;
  std::cerr << covariance_matrix[0] << " " << covariance_matrix[1] << std::endl;
  std::cerr << covariance_matrix[2] << " " << covariance_matrix[3] << std::endl;
  std::cerr << "Internal orientation: " << 0.5 * std::atan2(2 * covariance_matrix[1], covariance_matrix[0] - covariance_matrix[3]) << std::endl;
  orientation = orientation * 180 / M_PI; // Convert to degrees
  // Normalize to [0, 180)
  // Not to 0, 360, as the ellipse is symmetric and the orientation is defined as the angle between the semi-major axis and the x-axis
  orientation = std::fmod(orientation + 180, 180);
  while (orientation < 0) {
    orientation += 180;
  }
  while (orientation >= 180) {
    orientation -= 180;
  }
  return std::make_tuple(semi_major_axis, semi_minor_axis, orientation);
}

/**
 * @brief Set the Pos Confidence Ellipse object
 * 
 * @param position_confidence_ellipse 
 * @param covariance_matrix The four values of the covariance matrix in the order: cov_xx, cov_xy, cov_yx, cov_yy
 *                          The matrix has to be SPD, otherwise a std::invalid_argument exception is thrown.
 *                          Its coordinate system is aligned with the object (x = longitudinal, y = lateral)
 * @param object_heading The heading of the object in rad, with respect to WGS84
 */
template <typename PosConfidenceEllipse>
inline void setPosConfidenceEllipse(PosConfidenceEllipse& position_confidence_ellipse, const std::array<double, 4>& covariance_matrix, const double object_heading){
  auto [semi_major_axis, semi_minor_axis, orientation] = confidenceEllipseFromCovMatrix(covariance_matrix, object_heading);
  setPosConfidenceEllipse(position_confidence_ellipse, semi_major_axis, semi_minor_axis, orientation);
}


#endif  // ETSI_ITS_MSGS_UTILS_IMPL_CDD_CDD_SETTERS_COMMON_H