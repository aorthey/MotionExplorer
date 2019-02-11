#pragma once
#include <iostream>

//set neighborhood
//STATE_SPACE_UNKNOWN = 0,
//STATE_SPACE_REAL_VECTOR = 1,
//STATE_SPACE_SO2 = 2,
//STATE_SPACE_SO3 = 3,
//STATE_SPACE_SE2 = 4,
//STATE_SPACE_SE3 = 5,
//STATE_SPACE_TIME = 6,
//STATE_SPACE_DISCRETE = 7,
struct Neighborhood
{
  public:
    Neighborhood() = delete;
    Neighborhood(double);
    double WorkspaceDistanceToConfigurationSpaceDistance(double d);

  private:
    double c{1.0};
};
