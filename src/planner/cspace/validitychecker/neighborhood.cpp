#include "neighborhood.h"

Neighborhood::Neighborhood()
{
}

void Neighborhood::Init(){
  c = ComputeNeighborhoodConstant();
  if(c<=0){
    std::cout << "Neighborhood constant is non-positive: " << c << std::endl;
    exit(0);
  }
  std::cout << "init neighborhood scaled to " << c << std::endl;
}

double Neighborhood::WorkspaceDistanceToConfigurationSpaceDistance(double d)
{
  return c*d;
}
