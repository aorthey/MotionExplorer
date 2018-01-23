#include "multiquotient.h"
#include <ompl/util/Time.h>
#include <queue>

using namespace og;

template <class T>
MultiQuotient<T>::MultiQuotient(std::vector<ob::SpaceInformationPtr> &si_vec_, std::string type):
  ob::Planner(si_vec_.back(),"QMP"+type), si_vec(si_vec_)
{
  T::resetCounter();
  for(uint k = 0; k < si_vec.size(); k++){
    T* previous = nullptr;
    if(k>0) previous = quotientSpaces.back();

    T* ss = new T(si_vec.at(k), previous);
    quotientSpaces.push_back(ss);
  }

  std::cout << "Created hierarchy with " << si_vec.size() << " levels." << std::endl;
}

template <class T>
MultiQuotient<T>::~MultiQuotient(){
}

template <class T>
void MultiQuotient<T>::setup(){

  Planner::setup();
  for(uint k = 0; k < quotientSpaces.size(); k++){
    quotientSpaces.at(k)->setup();
  }
}

template <class T>
void MultiQuotient<T>::clear(){
  Planner::clear();
  solutions.clear();
  for(uint k = 0; k < quotientSpaces.size(); k++){
    quotientSpaces.at(k)->clear();
  }
  foundKLevelSolution = false;
}

template <class T>
ob::PlannerStatus MultiQuotient<T>::solve(const base::PlannerTerminationCondition &ptc){
  
  static const double T_GROW = 0.01; //time to grow before checking if solution exists

  auto cmp = [](T* left, T* right) 
              { 
                return left->getSamplingDensity() > right->getSamplingDensity();
              };

  std::priority_queue<T*, std::vector<T*>, decltype(cmp)> Q(cmp);

  for(uint k = 0; k < quotientSpaces.size(); k++){
    base::PathPtr sol_k;
    foundKLevelSolution = false;

    quotientSpaces.at(k)->Init();
    Q.push(quotientSpaces.at(k));

    base::PlannerTerminationCondition ptcOrSolutionFound([this, &ptc]
                                   { return ptc || foundKLevelSolution; });

    ompl::time::point t_k_start = ompl::time::now();
    while (!ptcOrSolutionFound())
    {
      T* jQuotient = Q.top();
      Q.pop();
      jQuotient->Grow(T_GROW);

      quotientSpaces.at(k)->checkForSolution(sol_k);

      if(quotientSpaces.at(k)->hasSolution()){
        solutions.push_back(sol_k);
        double t_k_end = ompl::time::seconds(ompl::time::now() - t_k_start);
        std::cout << "Found Solution on Level " << k << " after " << t_k_end << " seconds." << std::endl;
        foundKLevelSolution = true;
      }
      Q.push(jQuotient);
    }

    if(!foundKLevelSolution){
      std::cout << "could not find a solution on level " << k << std::endl;
      std::cout << "aborting" << std::endl;
      return ob::PlannerStatus::TIMEOUT;
    }
  }
  std::cout << "Found exact solution" << std::endl;

  base::PathPtr sol;
  quotientSpaces.back()->checkForSolution(sol);
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
void MultiQuotient<T>::setProblemDefinition(std::vector<ob::ProblemDefinitionPtr> &pdef_){
  pdef_vec = pdef_;
  ob::Planner::setProblemDefinition(pdef_vec.back());
  for(uint k = 0; k < pdef_vec.size(); k++){
    quotientSpaces.at(k)->setProblemDefinition(pdef_vec.at(k));
  }
}

template <class T>
void MultiQuotient<T>::setProblemDefinition(const ob::ProblemDefinitionPtr &pdef){

  //ob::ProblemDefinitionPtr pp = pdef.back();
  this->Planner::setProblemDefinition(pdef);
}

template <class T>
void MultiQuotient<T>::getPlannerData(ob::PlannerData &data) const{
  quotientSpaces.back()->getPlannerData(data);
}

