#include "prm_multislice.h"

using namespace og;
PRMMultiSlice::PRMMultiSlice(std::vector<ob::SpaceInformationPtr> &si_vec):
  ob::Planner(si_vec.back(),"PRMMultiSlice")
{
  //SpaceInformationPtr contains StateSpacePtr + ValidityChecker
  // and its native StateSampler, inherited by the OMPL StateSpace
  //
  // => to copy SI, we need to copy statespace and copy validitychecker

  for(uint k = 0; k < si_vec.size(); k++){
    PRMSliceNaive* previous = nullptr;
    if(k>0) previous = slicespaces.back();

    PRMSliceNaive* ss = new PRMSliceNaive(si_vec.at(k), previous);
    slicespaces.push_back(ss);
  }

  std::cout << "Created hierarchy with " << si_vec.size() << " levels." << std::endl;

}

PRMMultiSlice::~PRMMultiSlice(){
}


ob::PlannerStatus PRMMultiSlice::solve(const base::PlannerTerminationCondition &ptc){
  
  static const double ROADMAP_BUILD_TIME = 0.0001;

  for(uint k = 0; k < slicespaces.size(); k++){
    base::PathPtr sol_k;
    foundKLevelSolution = false;
    PRMSliceNaive *slice = slicespaces.at(k);
    slice->Init();

    base::PlannerTerminationCondition ptcOrSolutionFound([this, &ptc]
                                   { return ptc || foundKLevelSolution; });

    while (!ptcOrSolutionFound())
    {
      //TODO: priority queue instead, based on sampling density
      for(uint j = 0; j <= k; j++){
        PRMSliceNaive *jslice = slicespaces.at(j);
        jslice->Grow(ROADMAP_BUILD_TIME);
      }

      slice->checkForSolution(sol_k);

      if(slice->hasSolution()){
        foundKLevelSolution = true;
      }
    }
    std::cout << "vertices : " << slice->milestoneCount() << std::endl;

    if(foundKLevelSolution){
      std::cout << "found solution on level " << k << std::endl;
      std::cout << "continuing" << std::endl;
    }else{
      std::cout << "could not find a solution on level " << k << std::endl;
      std::cout << "aborting" << std::endl;
      return ob::PlannerStatus::TIMEOUT;
    }
  }
  return ob::PlannerStatus::EXACT_SOLUTION;
}

//void PRMMultiSlice::setup(){
//}

void PRMMultiSlice::setProblemDefinition(std::vector<ob::ProblemDefinitionPtr> &pdef){
  ob::ProblemDefinitionPtr pp = pdef.back();

  assert(pdef.size() == slicespaces.size());

  this->Planner::setProblemDefinition(pp);

  for(uint k = 0; k < pdef.size(); k++){
    slicespaces.at(k)->setProblemDefinition(pdef.at(k));
  }
}
void PRMMultiSlice::getPlannerData(ob::PlannerData &data) const{
  slicespaces.back()->getPlannerData(data);
}
