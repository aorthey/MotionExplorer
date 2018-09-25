#include "neighborhood.h"

Neighborhood::Neighborhood(double c_)
{
  this->c = c_;
}

double Neighborhood::WorkspaceDistanceToConfigurationSpaceDistance(double d)
{
  return this->c*d;
}

// void Neighborhood::SetConfigurationSpaceConstant(double c_)
// {
//   this->c = c_;
// }
// double Neighborhood::GetConfigurationSpaceConstant()
// {
//   return this->c;
// }
