#pragma once
#include <iostream>

struct Neighborhood
{
  public:
    Neighborhood();
    void Init();
    double WorkspaceDistanceToConfigurationSpaceDistance(double d);
    virtual double ComputeNeighborhoodConstant() = 0;
  private:
    double c{0};
};
