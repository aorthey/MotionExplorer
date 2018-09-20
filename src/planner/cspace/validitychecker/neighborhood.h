#pragma once
struct Neighborhood
{
  Neighborhood() = default;
  virtual double WorkspaceDistanceToConfigurationSpaceDistance(double) = 0;
};
