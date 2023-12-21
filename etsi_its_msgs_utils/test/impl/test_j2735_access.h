#pragma once

#include <cmath>
#include <limits>
#include <chrono>
#include <ctime>

#include <gtest/gtest.h>

#include "test_helpers.h"

TEST(etsi_its_msgs, test_j2735_access) {

  using namespace etsi_its_msgs::J2735_access;

  TestHelper helper;

  IntersectionGeometry intsct;

  unsigned int intersection_id = helper.randomInt(IntersectionID::MIN, IntersectionID::MAX);
  setIntersectionID(intsct, intersection_id);
  EXPECT_EQ(intersection_id, getIntersectionID(intsct));

  // Set specific position to test utm projection
  double latitude = 50.787467;
  double longitude = 6.046498;
  double altitude = 209.0;
  setPosition3D(intsct, latitude, longitude, altitude);
  int zone;
  bool northp;
  gm::PointStamped utm = getRefPointUTMPosition(intsct, zone, northp);
  EXPECT_NEAR(291827.02, utm.point.x, 1e-1);
  EXPECT_NEAR(5630349.72, utm.point.y, 1e-1);
  EXPECT_EQ(altitude, utm.point.z);
  EXPECT_EQ(32, zone);
  EXPECT_EQ(true, northp);
  setPosition3DFromUTMPosition(intsct.ref_point, utm, zone, northp);
  EXPECT_NEAR(latitude, getLatitude(intsct.ref_point), 1e-7);
  EXPECT_NEAR(longitude, getLongitude(intsct.ref_point), 1e-7);
  EXPECT_NEAR(altitude, getElevation(intsct.ref_point), 1e-2);
  
}