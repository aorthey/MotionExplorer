#include "prm_multislice.h"
#include <ompl/util/Time.h>
#include <queue>

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
  
  static const double ROADMAP_BUILD_TIME = 0.01;

  auto cmp = [](PRMSliceNaive* left, PRMSliceNaive* right) 
              { 
                return left->getSamplingDensity() > right->getSamplingDensity();
              };

  std::priority_queue<PRMSliceNaive*, std::vector<PRMSliceNaive*>, decltype(cmp)> Q(cmp);

  for(uint k = 0; k < slicespaces.size(); k++){
    base::PathPtr sol_k;
    foundKLevelSolution = false;
    PRMSliceNaive *kslice = slicespaces.at(k);
    kslice->Init();

    Q.push(kslice);

    base::PlannerTerminationCondition ptcOrSolutionFound([this, &ptc]
                                   { return ptc || foundKLevelSolution; });

    ompl::time::point t_k_start = ompl::time::now();
    while (!ptcOrSolutionFound())
    {
      PRMSliceNaive* jslice = Q.top();
      Q.pop();
      jslice->Grow(ROADMAP_BUILD_TIME);

      kslice->checkForSolution(sol_k);

      if(kslice->hasSolution()){
        double t_k_end = ompl::time::seconds(ompl::time::now() - t_k_start);
        std::cout << "Found Solution on Level " << k << " after " << t_k_end << " seconds." << std::endl;
        foundKLevelSolution = true;
      }
      Q.push(jslice);
    }
    std::cout << "vertices : " << kslice->milestoneCount() << std::endl;

    if(!foundKLevelSolution){
      std::cout << "could not find a solution on level " << k << std::endl;
      std::cout << "aborting" << std::endl;
      return ob::PlannerStatus::TIMEOUT;
    }
  }

//set pdef solution path!
  PRMSliceNaive *fullspace = slicespaces.back();
  base::PathPtr sol;
  fullspace->checkForSolution(sol);
  if (sol)
  {
    base::PlannerSolution psol(sol);
    psol.setPlannerName(getName());
    //psol.setOptimized(opt_, bestCost_, addedNewSolution());
    pdef_->addSolutionPath(psol);
  }

  return ob::PlannerStatus::EXACT_SOLUTION;
}

void PRMMultiSlice::setup(){
  Planner::setup();
  for(uint k = 0; k < slicespaces.size(); k++){
    PRMSliceNaive *sk = slicespaces.at(k);
    sk->setup();
  }
}

void PRMMultiSlice::setProblemDefinition(std::vector<ob::ProblemDefinitionPtr> &pdef){

  assert(pdef.size() == slicespaces.size());

  ob::ProblemDefinitionPtr pp = pdef.back();
  this->Planner::setProblemDefinition(pp);

  for(uint k = 0; k < pdef.size(); k++){
    slicespaces.at(k)->setProblemDefinition(pdef.at(k));
  }
}

void PRMMultiSlice::getPlannerData(ob::PlannerData &data) const{
  //Planner::getPlannerData(data);
  PRMSliceNaive *sb = slicespaces.back();
  sb->getPlannerData(data);
}

