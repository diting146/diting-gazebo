// Copyright 2026 diting
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <cmath>
#include <limits>
#include <vector>

#include "gtest/gtest.h"
#include "nav2_costmap_2d/cost_values.hpp"
#include "pb_omni_pid_pursuit_controller/footprint_collision.hpp"

namespace pb_omni_pid_pursuit_controller
{
namespace
{
std::vector<geometry_msgs::msg::Point> body()
{
  std::vector<geometry_msgs::msg::Point> result;
  // The same outward-rounded collision AABB and 1 cm padding as simulation.
  for (auto xy :
       {std::pair{-0.298, -0.277}, std::pair{0.298, -0.277}, std::pair{0.298, 0.277},
        std::pair{-0.298, 0.277}}) {
    geometry_msgs::msg::Point p;
    p.x = xy.first;
    p.y = xy.second;
    result.push_back(p);
  }
  return result;
}

TEST(FootprintCollision, AlignedBodyPassesEightHundredMillimeterCorridor)
{
  nav2_costmap_2d::Costmap2D map(120, 120, 0.01, -0.6, -0.6, 0);
  for (unsigned int x = 0; x < 120; ++x) {
    map.setCost(x, 19, 254);   // Upper edge y=-0.4.
    map.setCost(x, 100, 254);  // Lower edge y=+0.4.
  }
  EXPECT_FALSE(sweptFootprintCollision(map, body(), -0.2, 0, 0.2, 0, 0));
  EXPECT_TRUE(sweptFootprintCollision(map, body(), 0, 0.14, 0, 0.14, 0));
}

TEST(FootprintCollision, OrientationIsPhysicalBodyYaw)
{
  nav2_costmap_2d::Costmap2D map(200, 200, 0.01, -1, -1, 0);
  map.setCost(128, 100, 254);  // x=0.28 intersects the long side only.
  EXPECT_TRUE(sweptFootprintCollision(map, body(), 0, 0, 0, 0, 0));
  EXPECT_FALSE(sweptFootprintCollision(map, body(), 0, 0, 0, 0, M_PI / 2));
}

TEST(FootprintCollision, ObstacleInsideBodyIsNotMissedByAnEdgeOnlyChecker)
{
  nav2_costmap_2d::Costmap2D map(200, 200, 0.01, -1, -1, 0);
  map.setCost(100, 100, 254);
  EXPECT_TRUE(sweptFootprintCollision(map, body(), 0, 0, 0, 0, 0));
}

TEST(FootprintCollision, CornerOverlapAndCellAreaAreChecked)
{
  nav2_costmap_2d::Costmap2D map(200, 200, 0.01, -0.996, -0.996, 0);
  map.setCost(129, 127, 254);  // Corner cell: center is outside the body.
  EXPECT_TRUE(sweptFootprintCollision(map, body(), 0, 0, 0, 0, 0));
  map.setCost(129, 127, 0);
  map.setCost(130, 128, 254);
  EXPECT_FALSE(sweptFootprintCollision(map, body(), 0, 0, 0, 0, 0));
}

TEST(FootprintCollision, SweptSegmentCannotSkipAnObstacleBetweenPathSamples)
{
  nav2_costmap_2d::Costmap2D map(400, 200, 0.01, -2, -1, 0);
  map.setCost(200, 100, 254);
  EXPECT_FALSE(sweptFootprintCollision(map, body(), -1, 0, -1, 0, 0));
  EXPECT_FALSE(sweptFootprintCollision(map, body(), 1, 0, 1, 0, 0));
  EXPECT_TRUE(sweptFootprintCollision(map, body(), -1, 0, 1, 0, 0));
}

TEST(FootprintCollision, InflationIsNotAppliedTwiceButUnknownAndBoundsBlock)
{
  nav2_costmap_2d::Costmap2D map(200, 200, 0.01, -1, -1, 253);
  EXPECT_FALSE(sweptFootprintCollision(map, body(), 0, 0, 0, 0, 0));
  map.setCost(100, 100, 255);
  EXPECT_TRUE(sweptFootprintCollision(map, body(), 0, 0, 0, 0, 0));
  EXPECT_TRUE(sweptFootprintCollision(map, body(), 0.9, 0, 0.9, 0, 0));
}

TEST(FootprintCollision, InvalidGeometryFailsClosed)
{
  nav2_costmap_2d::Costmap2D map(200, 200, 0.01, -1, -1, 0);
  EXPECT_TRUE(sweptFootprintCollision(map, {}, 0, 0, 0, 0, 0));
  EXPECT_TRUE(
    sweptFootprintCollision(map, body(), 0, 0, 0, 0, std::numeric_limits<double>::quiet_NaN()));
}
}  // namespace
}  // namespace pb_omni_pid_pursuit_controller
