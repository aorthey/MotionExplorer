#include "prm_multiquotient.h"
#include <ompl/util/Time.h>
#include <queue>

using namespace og;

template <class T>
PRMMultiQuotient<T>::PRMMultiQuotient(std::vector<ob::SpaceInformationPtr> &si_vec_, std::string type):
  ob::Planner(si_vec_.back(),"QMP"+type), si_vec(si_vec_)
{
  T::resetCounter();
  for(uint k = 0; k < si_vec.size(); k++){
    T* previous = nullptr;
    if(k>0) previous = QuotientSpaces.back();

    T* ss = new T(si_vec.at(k), previous);
    QuotientSpaces.push_back(ss);
  }

  std::cout << "Created hierarchy with " << si_vec.size() << " levels." << std::endl;
}

template <class T>
PRMMultiQuotient<T>::~PRMMultiQuotient(){
}

template <class T>
void PRMMultiQuotient<T>::setup(){

  Planner::setup();
  for(uint k = 0; k < QuotientSpaces.size(); k++){
    T *sk = QuotientSpaces.at(k);
    sk->setup();
  }
}

template <class T>
void PRMMultiQuotient<T>::clear(){
  std::cout << "CLEAR MULTIQuotient" << std::endl;
  Planner::clear();
  solutions.clear();
  uint N = QuotientSpaces.size();
  for(uint k = 0; k < N; k++){
    QuotientSpaces.at(k)->clear();
  }
  foundKLevelSolution = false;
}

template <class T>
ob::PlannerStatus PRMMultiQuotient<T>::solve(const base::PlannerTerminationCondition &ptc){
  
  static const double ROADMAP_BUILD_TIME = 0.01;

  auto cmp = [](T* left, T* right) 
              { 
                return left->getSamplingDensity() > right->getSamplingDensity();
              };

  std::priority_queue<T*, std::vector<T*>, decltype(cmp)> Q(cmp);

  for(uint k = 0; k < QuotientSpaces.size(); k++){
    base::PathPtr sol_k;
    foundKLevelSolution = false;
    T *kQuotient = QuotientSpaces.at(k);
    kQuotient->Init();

    Q.push(kQuotient);

    base::PlannerTerminationCondition ptcOrSolutionFound([this, &ptc]
                                   { return ptc || foundKLevelSolution; });

    ompl::time::point t_k_start = ompl::time::now();
    while (!ptcOrSolutionFound())
    {
      T* jQuotient = Q.top();
      Q.pop();
      jQuotient->Grow(ROADMAP_BUILD_TIME);

      kQuotient->checkForSolution(sol_k);

      if(kQuotient->hasSolution()){
        solutions.push_back(sol_k);
        double t_k_end = ompl::time::seconds(ompl::time::now() - t_k_start);
        std::cout << "Found Solution on Level " << k << " after " << t_k_end << " seconds." << std::endl;
        foundKLevelSolution = true;
      }
      Q.push(jQuotient);
    }
    std::cout << "vertices : " << kQuotient->milestoneCount() << std::endl;

    if(!foundKLevelSolution){
      std::cout << "could not find a solution on level " << k << std::endl;
      std::cout << "aborting" << std::endl;
      return ob::PlannerStatus::TIMEOUT;
    }
  }
  std::cout << "Found exact solution" << std::endl;

//set pdef solution path!
  T *fullspace = QuotientSpaces.back();
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
void PRMMultiQuotient<T>::setProblemDefinition(std::vector<ob::ProblemDefinitionPtr> &pdef_){
  //assert(pdef.size() == QuotientSpaces.size());
  pdef_vec = pdef_;
  ob::Planner::setProblemDefinition(pdef_vec.back());
  for(uint k = 0; k < pdef_vec.size(); k++){
    QuotientSpaces.at(k)->setProblemDefinition(pdef_vec.at(k));
  }
}

template <class T>
void PRMMultiQuotient<T>::setProblemDefinition(const ob::ProblemDefinitionPtr &pdef){

  //ob::ProblemDefinitionPtr pp = pdef.back();
  this->Planner::setProblemDefinition(pdef);
}

template <class T>
void PRMMultiQuotient<T>::getPlannerData(ob::PlannerData &data) const{
  T *lastQuotient = QuotientSpaces.back();
  lastQuotient->getPlannerData(data);
}

