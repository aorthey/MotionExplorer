#include "prm_multislice.h"

using namespace og;
PRMMultiSlice::PRMMultiSlice(std::vector<ob::SpaceInformationPtr> &si_vec):
  ob::Planner(si_vec.at(0),"PRMMultiSlice")
{
  //SpaceInformationPtr contains StateSpacePtr + ValidityChecker
  // and its native StateSampler, inherited by the OMPL StateSpace
  //
  // => to copy SI, we need to copy statespace and copy validitychecker

  PRMSliceNaive* s0 = new PRMSliceNaive(si_vec.at(0), nullptr);
  slicespaces.push_back(s0);

  for(uint k = 1; k < si_vec.size(); k++){
    PRMSliceNaive* ss = new PRMSliceNaive(si_vec.at(k), slicespaces.back());
    slicespaces.push_back(ss);
  }

  std::cout << "Created hierarchy with " << si_vec.size() << " levels." << std::endl;

}

PRMMultiSlice::~PRMMultiSlice(){
}
ob::PlannerStatus PRMMultiSlice::solve(const base::PlannerTerminationCondition &ptc){
  
  static const double ROADMAP_BUILD_TIME = 0.01;

  for(uint k = 0; k < slicespaces.size(); k++){
    base::PathPtr sol_k;
    foundKLevelSolution = false;
    PRMSliceNaive *slice = slicespaces.at(k);

    base::PlannerTerminationCondition ptcOrSolutionFound([this, &ptc]
                                   { return ptc || foundKLevelSolution; });

    while (!ptcOrSolutionFound())
    {
      slice->Grow(ROADMAP_BUILD_TIME);
      slice->checkForSolution(sol_k);
      if(slice->hasSolution()){
        foundKLevelSolution = true;
      }
    }

    if(foundKLevelSolution){
      std::cout << "found solution on level " << k << std::endl;
    }else{
      std::cout << "could not find a solution on level " << k << std::endl;
      std::cout << "aborting" << std::endl;
    }
    exit(0);
  }
}

void PRMMultiSlice::setup(){
}

