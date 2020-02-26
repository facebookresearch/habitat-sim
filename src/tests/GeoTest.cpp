// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include <gtest/gtest.h>
#include "esp/geo/CoordinateFrame.h"
#include "esp/geo/OBB.h"
#include "esp/geo/geo.h"

using namespace esp;
using namespace esp::geo;

TEST(GeoTest, OBBConstruction) {
  OBB obb1;
  // LOG(INFO) << obb1;
  const vec3f center(0, 0, 0);
  const vec3f dimensions(20, 2, 10);
  const quatf rot1 = quatf::FromTwoVectors(vec3f::UnitY(), vec3f::UnitZ());
  OBB obb2(center, dimensions, rot1);
  // LOG(INFO) << obb2;
  EXPECT_TRUE(obb2.center().isApprox(center));
  EXPECT_TRUE(obb2.sizes().isApprox(dimensions));
  EXPECT_TRUE(obb2.halfExtents().isApprox(dimensions / 2));
  EXPECT_TRUE(obb2.rotation().coeffs().isApprox(rot1.coeffs()));

  box3f aabb(vec3f(-1, -2, -3), vec3f(3, 2, 1));
  OBB obb3(aabb);
  EXPECT_TRUE(obb3.center().isApprox(aabb.center()));
  EXPECT_TRUE(obb3.toAABB().isApprox(aabb));
}

TEST(GeoTest, OBBfunctions) {
  const vec3f center(0, 0, 0);
  const vec3f dimensions(20, 2, 10);
  const quatf rot1 = quatf::FromTwoVectors(vec3f::UnitY(), vec3f::UnitZ());
  OBB obb2(center, dimensions, rot1);
  EXPECT_TRUE(obb2.contains(vec3f(0, 0, 0)));
  EXPECT_TRUE(obb2.contains(vec3f(-5, -2, 0.5)));
  EXPECT_TRUE(obb2.contains(vec3f(5, 0, -0.5)));
  EXPECT_TRUE(!obb2.contains(vec3f(5, 0, 2)));
  EXPECT_TRUE(!obb2.contains(vec3f(-10, 0.5, -2)));

  const box3f aabb = obb2.toAABB();
  // LOG(INFO) << aabb;
  EXPECT_FLOAT_EQ(aabb.min().x(), -10);
  EXPECT_FLOAT_EQ(aabb.min().y(), -5);
  EXPECT_FLOAT_EQ(aabb.min().z(), -1);
  EXPECT_FLOAT_EQ(aabb.max().x(), 10);
  EXPECT_FLOAT_EQ(aabb.max().y(), 5);
  EXPECT_FLOAT_EQ(aabb.max().z(), 1);

  const Transform identity = obb2.worldToLocal() * obb2.localToWorld();
  EXPECT_TRUE(identity.isApprox(Transform::Identity()));
  EXPECT_TRUE(obb2.contains(obb2.localToWorld() * vec3f(0, 0, 0)));
  EXPECT_TRUE(obb2.contains(obb2.localToWorld() * vec3f(1, -1, 1)));
  EXPECT_TRUE(obb2.contains(obb2.localToWorld() * vec3f(-1, -1, -1)));
  EXPECT_TRUE(!obb2.contains(obb2.localToWorld() * vec3f(-1, -1.5, -1)));
  EXPECT_FLOAT_EQ(obb2.distance(vec3f(-20, 0, 0)), 10);
  EXPECT_FLOAT_EQ(obb2.distance(vec3f(-10, -5, 2)), 1);
}

TEST(GeoTest, CoordinateFrame) {
  const vec3f origin(1, -2, 3);
  const vec3f up(0, 0, 1);
  const vec3f front(-1, 0, 0);
  quatf rotation = quatf::FromTwoVectors(ESP_UP, up) *
                   quatf::FromTwoVectors(ESP_FRONT, front);
  Transform xform;
  xform.rotate(rotation);
  xform.translate(origin);

  CoordinateFrame c1(up, front, origin);
  EXPECT_TRUE(c1.up().isApprox(up));
  EXPECT_TRUE(c1.gravity().isApprox(-up));
  EXPECT_TRUE(c1.front().isApprox(front));
  EXPECT_TRUE(c1.back().isApprox(-front));
  EXPECT_TRUE(c1.up().isApprox(rotation * ESP_UP));
  EXPECT_TRUE(c1.front().isApprox(rotation * ESP_FRONT));
  EXPECT_TRUE(c1.origin().isApprox(origin));
  EXPECT_TRUE(c1.rotationWorldToFrame().isApprox(rotation));

  CoordinateFrame c2(rotation, origin);
  EXPECT_EQ(c1, c2);
  EXPECT_TRUE(c2.up().isApprox(up));
  EXPECT_TRUE(c2.gravity().isApprox(-up));
  EXPECT_TRUE(c2.front().isApprox(front));
  EXPECT_TRUE(c2.back().isApprox(-front));
  EXPECT_TRUE(c2.up().isApprox(rotation * ESP_UP));
  EXPECT_TRUE(c2.front().isApprox(rotation * ESP_FRONT));
  EXPECT_TRUE(c2.origin().isApprox(origin));
  EXPECT_TRUE(c2.rotationWorldToFrame().isApprox(rotation));

  const std::string j = R"({"up":[0,0,1],"front":[-1,0,0],"origin":[1,-2,3]})";
  EXPECT_EQ(c1.toJson(), j);
  CoordinateFrame c3(j);
  EXPECT_EQ(c1, c3);
  CoordinateFrame c4;
  c4.fromJson(j);
  EXPECT_EQ(c3, c4);
}
