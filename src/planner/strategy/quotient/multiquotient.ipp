#include "elements/plannerdata_vertex_annotated.h"
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/spaces/SO3StateSpace.h>
#include <ompl/util/Time.h>
#include <queue>

using namespace og;

template <class T, class Tlast>
MultiQuotient<T,Tlast>::MultiQuotient(std::vector<ob::SpaceInformationPtr> &si_vec_, std::string type):
  ob::Planner(si_vec_.back(), type), si_vec(si_vec_)
{
  T::resetCounter();
  for(uint k = 0; k < si_vec.size(); k++){
    og::Quotient* parent = nullptr;
    if(k>0) parent = quotientSpaces.back();

    if(k>0 && k>=si_vec.size()-1){
      Tlast* ss = new Tlast(si_vec.at(k), parent);
      quotientSpaces.push_back(ss);
    }else{
      T* ss = new T(si_vec.at(k), parent);
      quotientSpaces.push_back(ss);
    }
  }
  std::cout << "Created hierarchy with " << si_vec.size() << " levels." << std::endl;
}

template <class T, class Tlast>
MultiQuotient<T,Tlast>::~MultiQuotient(){
}

template <class T, class Tlast>
void MultiQuotient<T,Tlast>::setup(){

  Planner::setup();
  for(uint k = 0; k < quotientSpaces.size(); k++){
    quotientSpaces.at(k)->setup();
  }
  currentQuotientLevel = 0;
}

template <class T, class Tlast>
void MultiQuotient<T,Tlast>::clear(){
  Planner::clear();
  for(uint k = 0; k < quotientSpaces.size(); k++){
    quotientSpaces.at(k)->clear();
  }
  while(!Q.empty()) Q.pop();

  foundKLevelSolution = false;

  solutions.clear();
  pdef_->clearSolutionPaths();
  for(uint k = 0; k < pdef_vec.size(); k++){
    pdef_vec.at(k)->clearSolutionPaths();
  }
  currentQuotientLevel = 0;
  

}

template <class T, class Tlast>
ob::PlannerStatus MultiQuotient<T,Tlast>::solve(const base::PlannerTerminationCondition &ptc)
{
  
  static const double T_GROW = 0.01; //time to grow before Checking if solution exists

  //auto cmp = [](og::Quotient* left, og::Quotient* right) 
  //            { 
  //              return left->GetImportance() < right->GetImportance();
  //            };

  //std::priority_queue<og::Quotient*, std::vector<og::Quotient*>, decltype(cmp)> Q(cmp);

  const bool DEBUG = true;
  ompl::time::point t_start = ompl::time::now();

  for(uint k = currentQuotientLevel; k < quotientSpaces.size(); k++){
    base::PathPtr sol_k;
    foundKLevelSolution = false;

    Q.push(quotientSpaces.at(k));

    base::PlannerTerminationCondition ptcOrSolutionFound([this, &ptc]
                                   { return ptc || foundKLevelSolution; });

    while (!ptcOrSolutionFound())
    {
      og::Quotient* jQuotient = Q.top();
      Q.pop();
      jQuotient->Grow(T_GROW);
      bool hasSolution = quotientSpaces.at(k)->GetSolution(sol_k);

      if(hasSolution){
        solutions.push_back(sol_k);
        if(DEBUG){
          double t_k_end = ompl::time::seconds(ompl::time::now() - t_start);
          std::cout << std::string(80, '#') << std::endl;
          std::cout << "Found Solution on Level " << k << " after " << t_k_end << " seconds." << std::endl;
          std::cout << std::string(80, '#') << std::endl;
          std::cout << *quotientSpaces.at(k) << std::endl;
        }
        foundKLevelSolution = true;
        currentQuotientLevel = k+1;
        if(currentQuotientLevel < quotientSpaces.size()-1){
          quotientSpaces.at(k+1)->clear();
        }
      }
      Q.push(jQuotient);
    }

    if(!foundKLevelSolution){
      if(DEBUG){
        std::cout << std::string(80, '#') << std::endl;
        std::cout << "could not find a solution on level " << k << std::endl;
        std::cout << std::string(80, '#') << std::endl;
        for(uint i = 0; i < k+1; i++){
          std::cout << *quotientSpaces.at(i) << std::endl;
        }
      }
      return ob::PlannerStatus::TIMEOUT;
    }
  }
  if(DEBUG){
    double t_end = ompl::time::seconds(ompl::time::now() - t_start);
    std::cout << std::string(80, '#') << std::endl;
    std::cout << "Found exact solution after " << t_end << " seconds." << std::endl;
    std::cout << std::string(80, '#') << std::endl;
  }

  base::PathPtr sol;
  if(quotientSpaces.back()->GetSolution(sol))
  {
    base::PlannerSolution psol(sol);
    psol.setPlannerName(getName());
    pdef_->addSolutionPath(psol);
  }

  return ob::PlannerStatus::EXACT_SOLUTION;
}



template <class T, class Tlast>
void MultiQuotient<T,Tlast>::setProblemDefinition(std::vector<ob::ProblemDefinitionPtr> &pdef_)
{
  pdef_vec = pdef_;
  ob::Planner::setProblemDefinition(pdef_vec.back());
  for(uint k = 0; k < pdef_vec.size(); k++){
    quotientSpaces.at(k)->setProblemDefinition(pdef_vec.at(k));
  }
}

template <class T, class Tlast>
void MultiQuotient<T,Tlast>::setProblemDefinition(const ob::ProblemDefinitionPtr &pdef)
{
  //ob::ProblemDefinitionPtr pp = pdef.back();
  this->Planner::setProblemDefinition(pdef);
}

template <class T, class Tlast>
void MultiQuotient<T,Tlast>::getPlannerData(ob::PlannerData &data) const
{
  uint Nvertices = data.numVertices();
  if(Nvertices>0){
    std::cout << "cannot get planner data if plannerdata is already populated" << std::endl;
    std::cout << "PlannerData has " << Nvertices << " vertices." << std::endl;
    exit(0);
  }

  uint K = min(solutions.size()+1,quotientSpaces.size());

  for(uint k = 0; k < K; k++){
    og::Quotient *Qk = quotientSpaces.at(k);
    Qk->getPlannerData(data);

    //label all new vertices
    uint ctr = 0;
    for(uint vidx = Nvertices; vidx < data.numVertices(); vidx++){
      PlannerDataVertexAnnotated &v = *static_cast<PlannerDataVertexAnnotated*>(&data.getVertex(vidx));
      v.SetLevel(k);
      v.SetPath( std::vector<int>(k+1));
      v.SetMaxLevel(K);

      const ob::State *s_V = v.getState();
      ob::State *s_Q0 = Qk->getSpaceInformation()->cloneState(s_V);

      for(uint m = k+1; m < quotientSpaces.size(); m++){
        og::Quotient *Qm = quotientSpaces.at(m);
        ob::State *s_X1 = Qm->GetX1()->allocState();
        ob::State *s_Q1 = Qm->getSpaceInformation()->allocState();
        if(Qm->GetX1()->getStateSpace()->getType() == ob::STATE_SPACE_SO3) {
          static_cast<ob::SO3StateSpace::StateType*>(s_X1)->setIdentity();
        }
        if(Qm->GetX1()->getStateSpace()->getType() == ob::STATE_SPACE_SO2) {
          static_cast<ob::SO2StateSpace::StateType*>(s_X1)->setIdentity();
        }
        //Qm->SampleX1(s_X1);
        Qm->MergeStates(s_Q0, s_X1, s_Q1);
        quotientSpaces.at(m-1)->getSpaceInformation()->freeState(s_Q0);
        Qm->GetX1()->freeState(s_X1);
        s_Q0 = s_Q1;
      }
      v.setState(s_Q0);
      ctr++;
    }
    Nvertices = data.numVertices();
  }

  // for(uint vidx = 0; vidx < data.numVertices(); vidx++){
  //   PlannerDataVertexAnnotated &v = *static_cast<PlannerDataVertexAnnotated*>(&data.getVertex(vidx));
  //   std::cout << "[MultiQuotient] vertex " << vidx << " " << v.GetOpenNeighborhoodDistance() << std::endl;
  // }
  //exit(0);
}
