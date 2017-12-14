#include "prm_multislice.h"
#include <ompl/util/Time.h>
#include <queue>

using namespace og;

template <class T>
PRMMultiSlice<T>::PRMMultiSlice(std::vector<ob::SpaceInformationPtr> &si_vec_, std::string type):
  ob::Planner(si_vec_.back(),"PRMMultiSlice"+type), si_vec(si_vec_)
{
  //SpaceInformationPtr contains StateSpacePtr + ValidityChecker
  // and its native StateSampler, inherited by the OMPL StateSpace
  //
  // => to copy SI, we need to copy statespace and copy validitychecker
    T::resetCounter();
    for(uint k = 0; k < si_vec.size(); k++){
      T* previous = nullptr;
      if(k>0) previous = slicespaces.back();

      T* ss = new T(si_vec.at(k), previous);
      slicespaces.push_back(ss);
    }

    std::cout << "Created hierarchy with " << si_vec.size() << " levels." << std::endl;

}

template <class T>
PRMMultiSlice<T>::~PRMMultiSlice(){
}
template <class T>
void PRMMultiSlice<T>::setup(){

  Planner::setup();
  for(uint k = 0; k < slicespaces.size(); k++){
    T *sk = slicespaces.at(k);
    sk->setup();
  }
}

template <class T>
void PRMMultiSlice<T>::clear(){
  std::cout << "CLEAR MULTISLICE" << std::endl;
  Planner::clear();
  solutions.clear();
  uint N = slicespaces.size();
  for(uint k = 0; k < N; k++){
    slicespaces.at(k)->clear();
  }
  foundKLevelSolution = false;
}

template <class T>
ob::PlannerStatus PRMMultiSlice<T>::solve(const base::PlannerTerminationCondition &ptc){
  
  static const double ROADMAP_BUILD_TIME = 0.01;

  auto cmp = [](T* left, T* right) 
              { 
                return left->getSamplingDensity() > right->getSamplingDensity();
              };

  std::priority_queue<T*, std::vector<T*>, decltype(cmp)> Q(cmp);

  for(uint k = 0; k < slicespaces.size(); k++){
    base::PathPtr sol_k;
    foundKLevelSolution = false;
    T *kslice = slicespaces.at(k);
    kslice->Init();

    Q.push(kslice);

    base::PlannerTerminationCondition ptcOrSolutionFound([this, &ptc]
                                   { return ptc || foundKLevelSolution; });

    ompl::time::point t_k_start = ompl::time::now();
    while (!ptcOrSolutionFound())
    {
      T* jslice = Q.top();
      Q.pop();
      jslice->Grow(ROADMAP_BUILD_TIME);

      kslice->checkForSolution(sol_k);

      if(kslice->hasSolution()){
        solutions.push_back(sol_k);
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
  std::cout << "Found exact solution" << std::endl;

//set pdef solution path!
  T *fullspace = slicespaces.back();
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


template <class T>
void PRMMultiSlice<T>::setProblemDefinition(std::vector<ob::ProblemDefinitionPtr> &pdef_){
  //assert(pdef.size() == slicespaces.size());
  pdef_vec = pdef_;
  ob::Planner::setProblemDefinition(pdef_vec.back());
  for(uint k = 0; k < pdef_vec.size(); k++){
    slicespaces.at(k)->setProblemDefinition(pdef_vec.at(k));
  }
}

template <class T>
void PRMMultiSlice<T>::setProblemDefinition(const ob::ProblemDefinitionPtr &pdef){

  //ob::ProblemDefinitionPtr pp = pdef.back();
  this->Planner::setProblemDefinition(pdef);

}

template <class T>
void PRMMultiSlice<T>::getPlannerData(ob::PlannerData &data) const{
  //Planner::getPlannerData(data);
  T *sb = slicespaces.back();
  sb->getPlannerData(data);
}

