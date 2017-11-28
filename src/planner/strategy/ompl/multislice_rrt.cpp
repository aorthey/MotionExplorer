#include "multislice_rrt.h"
#include "slice_rrt.h"
#include <ompl/base/Planner.h>

MultiSliceRRT::MultiSliceRRT(const std::vector<ob::SpaceInformationPtr> Cvec, const std::vector<ob::SpaceInformationPtr> Mvec):
{
  std::cout << "creating slicespaces" << std::endl;

  slicespaces.push_back( new SliceRRT(Cvec.at(0), Mvec.at(0)) );
  for(uint k = 1; k < Cvec.size(); k++){
    slicespaces.push_back( new SliceRRT(Cvec.at(k), Mvec.at(k), slicespaces.back()) );
  }
}

~MultiSliceRRT::MultiSliceRRT(void){
}

base::PlannerStatus MultiSliceRRT::solve(const ob::PlannerTerminationCondition &ptc){
}
void MultiSliceRRT::clear(void) {
}
void MultiSliceRRT::setup(void) {
}
void MultiSliceRRT::getPlannerData(ob::PlannerData &data) const {
}
