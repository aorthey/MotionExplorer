#include "multiquotient.h"
#include "elements/plannerdata_vertex_annotated.h"
#include <ompl/util/Time.h>
#include <queue>

using namespace og;

template <class T, class Tlast>
MultiQuotient<T,Tlast>::MultiQuotient(std::vector<ob::SpaceInformationPtr> &si_vec_, std::string type):
  ob::Planner(si_vec_.back(),"QMP"+type), si_vec(si_vec_)
{
  T::resetCounter();
  for(uint k = 0; k < si_vec.size(); k++){
    og::Quotient* previous = nullptr;
    if(k>0) previous = quotientSpaces.back();

    if(k>0 && k>=si_vec.size()-1){
      Tlast* ss = new Tlast(si_vec.at(k), previous);
      quotientSpaces.push_back(ss);
    }else{
      T* ss = new T(si_vec.at(k), previous);
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
}

template <class T, class Tlast>
void MultiQuotient<T,Tlast>::clear(){
  Planner::clear();
  solutions.clear();
  for(uint k = 0; k < quotientSpaces.size(); k++){
    quotientSpaces.at(k)->clear();
  }
  foundKLevelSolution = false;
}

void PrintQuotientSpaces(std::vector<Quotient*> quotientSpaces, uint k=0){
  if(k<=0) k=quotientSpaces.size()-1;
  for(uint i = 0; i <= k; i++){
    og::Quotient *Qi = quotientSpaces.at(i);
    std::cout << ">> level " << i << " vertices " << Qi->GetNumberOfVertices() 
      << " edges " << Qi->GetNumberOfEdges() 
      << " density " << Qi->GetSamplingDensity() << std::endl;
  }
}

template <class T, class Tlast>
ob::PlannerStatus MultiQuotient<T,Tlast>::solve(const base::PlannerTerminationCondition &ptc){
  
  static const double T_GROW = 0.001; //time to grow before Checking if solution exists

  auto cmp = [](og::Quotient* left, og::Quotient* right) 
              { 
                return left->GetSamplingDensity() > right->GetSamplingDensity();
              };

  std::priority_queue<og::Quotient*, std::vector<og::Quotient*>, decltype(cmp)> Q(cmp);

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
      og::Quotient* jQuotient = Q.top();
      Q.pop();
      jQuotient->Grow(T_GROW);
      quotientSpaces.at(k)->CheckForSolution(sol_k);

      if(quotientSpaces.at(k)->HasSolution()){
        solutions.push_back(sol_k);
        double t_k_end = ompl::time::seconds(ompl::time::now() - t_k_start);
        std::cout << "Found Solution on Level " << k << " after " << t_k_end << " seconds." << std::endl;
        //PrintQuotientSpaces(quotientSpaces, k);
        foundKLevelSolution = true;
      }
      Q.push(jQuotient);
    }

    if(!foundKLevelSolution){
      std::cout << "could not find a solution on level " << k << std::endl;
      //PrintQuotientSpaces(quotientSpaces, k);
      return ob::PlannerStatus::TIMEOUT;
    }
  }
  std::cout << "Found exact solution" << std::endl;
  PrintQuotientSpaces(quotientSpaces);

  base::PathPtr sol;
  quotientSpaces.back()->CheckForSolution(sol);
  if (sol)
  {
    base::PlannerSolution psol(sol);
    psol.setPlannerName(getName());
    pdef_->addSolutionPath(psol);
  }

  return ob::PlannerStatus::EXACT_SOLUTION;
}



template <class T, class Tlast>
void MultiQuotient<T,Tlast>::setProblemDefinition(std::vector<ob::ProblemDefinitionPtr> &pdef_){
  pdef_vec = pdef_;
  ob::Planner::setProblemDefinition(pdef_vec.back());
  for(uint k = 0; k < pdef_vec.size(); k++){
    quotientSpaces.at(k)->setProblemDefinition(pdef_vec.at(k));
  }
}

template <class T, class Tlast>
void MultiQuotient<T,Tlast>::setProblemDefinition(const ob::ProblemDefinitionPtr &pdef){

  //ob::ProblemDefinitionPtr pp = pdef.back();
  this->Planner::setProblemDefinition(pdef);
}

template <class T, class Tlast>
void MultiQuotient<T,Tlast>::getPlannerData(ob::PlannerData &data) const{
  uint Nvertices = data.numVertices();
  if(Nvertices>0){
    std::cout << "cannot get planner data if plannerdata is already populated" << std::endl;
    std::cout << "PlannerData has " << Nvertices << " vertices." << std::endl;
    exit(0);
  }


  uint K = min(solutions.size()+1,quotientSpaces.size());

  for(uint k = 0; k < K; k++){
    og::Quotient *Qk = quotientSpaces.at(k);
    //get all vertices
    Qk->getPlannerData(data);

    //remove start and goal vertices for all expect the last space
    if(k<K-1){
      uint Nstart = data.numStartVertices();
      uint Ngoal = data.numGoalVertices();
      for(uint i = 0; i < Nstart; i++){
        uint sidx = data.getStartIndex(i);
        data.removeVertex(sidx);
      }
      for(uint i = 0; i < Ngoal; i++){
        uint sidx = data.getGoalIndex(i);
        data.removeVertex(sidx);
      }
    }

    //label all new vertices
    uint ctr = 0;
    for(uint vidx = Nvertices; vidx < data.numVertices(); vidx++){
      PlannerDataVertexAnnotated &v = *static_cast<PlannerDataVertexAnnotated*>(&data.getVertex(vidx));
      v.SetLevel(k);
      v.SetMaxLevel(K);

      const ob::State *s_V = v.getState();
      ob::State *s_M0 = Qk->getSpaceInformation()->cloneState(s_V);

      for(uint m = k+1; m < quotientSpaces.size(); m++){
        og::Quotient *Qm = quotientSpaces.at(m);
        ob::State *s_C1 = Qm->getC1()->allocState();
        ob::State *s_M1 = Qm->getSpaceInformation()->allocState();
        Qm->SampleC1(s_C1);
        Qm->mergeStates(s_M0, s_C1, s_M1);
        Qm->getC1()->freeState(s_M0);
        Qm->getC1()->freeState(s_C1);
        //Qm->getSpaceInformation()->printState(s_M1);
        s_M0 = s_M1;
      }
      v.setState(s_M0);
      ctr++;
    }
    std::cout << "multiquotient: added " << ctr << " vertices on level " << k << std::endl;
    Nvertices = data.numVertices();
  }
  //quotientSpaces.back()->getPlannerData(data);
}

