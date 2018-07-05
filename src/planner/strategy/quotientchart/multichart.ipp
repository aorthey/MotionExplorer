#include "multichart.h"
#include "elements/plannerdata_vertex_annotated.h"
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/spaces/SO3StateSpace.h>
#include <ompl/util/Time.h>
#include <queue>

using namespace og;

template <class T>
MultiChart<T>::MultiChart(std::vector<ob::SpaceInformationPtr> &si_vec_, std::string type):
  ob::Planner(si_vec_.back(), type), si_vec(si_vec_)
{
  T::resetCounter();
  levels = si_vec.size();
  root = new T(si_vec.at(0), nullptr);
  root->SetLevel(0);

  for(uint k = 0; k < si_vec.size(); k++){
    og::QuotientChart* parent = nullptr;
    if(k>0) parent = quotientSpaces.back();
    T* ss = new T(si_vec.at(k), parent);
    quotientSpaces.push_back(ss);
  }
}

template <class T>
MultiChart<T>::~MultiChart(){
}

template <class T>
void MultiChart<T>::setup(){
  Planner::setup();
  root->setup();
}

template <class T>
void MultiChart<T>::clear(){
  Planner::clear();
  root->clear();
}

//void PrintQuotientSpaces(std::vector<Quotient*> quotientSpaces, uint k=0)
//{
//  return;
//  if(k<=0) k=quotientSpaces.size()-1;
//  uint total_vertices = 0;
//  uint total_sampled_vertices = 0;
//  for(uint i = 0; i <= k; i++){
//    og::Quotient *Qi = quotientSpaces.at(i);
//    if(Qi->GetNumberOfSampledVertices()<=0) continue;
//    uint N = 6;
//    std::cout 
//      << ">> level " << i << ": " 
//      << std::setw(N)
//      << std::setfill(' ')
//      << Qi->GetNumberOfVertices() 
//      << " vertices ("
//      << std::setw(N)
//      << std::setfill(' ')
//      << Qi->GetNumberOfSampledVertices() 
//      << " sampled) |"
//      << std::setw(N)
//      << std::setfill(' ')
//      << Qi->GetNumberOfEdges() 
//      << " edges | " 
//      << Qi->GetSamplingDensity() 
//      << " density \n";
//    total_vertices += Qi->GetNumberOfVertices();
//    total_sampled_vertices += Qi->GetNumberOfSampledVertices();
//  }
//  std::cout << std::string(80, '-') << std::endl;
//  double total_rejected = 1.0-(double)total_vertices/(double)total_sampled_vertices;
//  std::cout << ">> total: " << total_vertices << " vertices ("<<total_sampled_vertices << " sampled, " << setprecision(2) << total_rejected << " percent rejected)" << std::endl;
//  std::cout << std::string(80, '-') << std::endl;
//}

template <class T>
ob::PlannerStatus MultiChart<T>::solve(const base::PlannerTerminationCondition &ptc)
{
  static const double T_GROW = 0.01; //time to grow before Checking if solution exists

  auto cmp = [](og::QuotientChart* left, og::QuotientChart* right) 
              { 
                return left->GetImportance() > right->GetImportance();
              };

  std::priority_queue<og::QuotientChart*, std::vector<og::QuotientChart*>, decltype(cmp)> Q(cmp);

  //ompl::time::point t_start = ompl::time::now();
  bool found_path_on_last_level = false;

  base::PlannerTerminationCondition ptcOrSolutionFound([this, &ptc]
                                 { return ptc || foundKLevelSolution; });

  root->Init();
  Q.push(root);
  og::QuotientChart* current_node = root;

  //each chart has an associated importance weight. If we operate on a certain
  //path [i,j,k], then the charts Q_i, Q_ij and Q_ijk have the usual importance
  //as in the quotientspace approach. All other charts are set to zero.
  while(!ptcOrSolutionFound)
  {
    //growing the charts occurs in the path below current_node.
    og::QuotientChart* jChart = Q.top();
    Q.pop();
    jChart->Grow(T_GROW);

    if(current_node->FoundNewPath()){
      if(current_node->GetLevel() == levels-1){
        found_path_on_last_level = true;
      }else{
        //not yet reached maximal level. create a new sibling chart (containing
        //the solution path plus associated vertices), and create
        //a child of the sibling (the nullspace of the solution path plus its
        //associated vertices on the next quotientchart).
        std::cout << "found path on level " << current_node->GetLevel() << std::endl;
        std::cout << "Trying to extract path subgraph" << std::endl;
        og::QuotientChart *local = new og::QuotientChart(si_vec.at(current_node->GetLevel()), 
            static_cast<og::QuotientChart*>(current_node->GetParent()));
        //Graph G = current_node->GetPathSubgraph( current_node->GetNumberOfPaths() - 1 ); //get last path
        //local->SetGraph(G);

        og::QuotientChart *global = new og::QuotientChart(si_vec.at(current_node->GetLevel()+1), local);
        current_node->AddSibling(local);
        Q.push(global);
        exit(0);
      }
    }

    Q.push(jChart);
  }
  if(found_path_on_last_level)
  {
    base::PathPtr sol;
    current_node->CheckForSolution(sol);
    if (sol)
    {
      base::PlannerSolution psol(sol);
      psol.setPlannerName(getName());
      pdef_->addSolutionPath(psol);
    }else{
      std::cout << "error: expected a solution path" << std::endl;
      exit(0);
    }

    return ob::PlannerStatus::EXACT_SOLUTION;
  }
  return ob::PlannerStatus::TIMEOUT;

}



template <class T>
void MultiChart<T>::setProblemDefinition(std::vector<ob::ProblemDefinitionPtr> &pdef_)
{
  pdef_vec = pdef_;
  ob::Planner::setProblemDefinition(pdef_vec.back());
  root->setProblemDefinition(pdef_vec.at(0));
}

template <class T>
void MultiChart<T>::setProblemDefinition(const ob::ProblemDefinitionPtr &pdef)
{
  this->Planner::setProblemDefinition(pdef);
}

template <class T>
void MultiChart<T>::getPlannerData(ob::PlannerData &data) const
{
  uint Nvertices = data.numVertices();
  if(Nvertices>0){
    std::cout << "cannot get planner data if plannerdata is already populated" << std::endl;
    std::cout << "PlannerData has " << Nvertices << " vertices." << std::endl;
    exit(0);
  }
  QuotientChart *current = root;
  current->getPlannerData(data);

  for(uint vidx = 0; vidx < data.numVertices(); vidx++)
  {
    PlannerDataVertexAnnotated &v = *static_cast<PlannerDataVertexAnnotated*>(&data.getVertex(vidx));
    v.SetMaxLevel(levels);
    uint k = v.GetLevel();

    const ob::State *s_V = v.getState();
    ob::State *s_M0 = si_vec.at(k)->cloneState(s_V);

    for(uint m = k+1; m < levels; m++){
      og::QuotientChart *Qm = quotientSpaces.at(m);
      ob::State *s_C1 = Qm->getC1()->allocState();
      ob::State *s_M1 = Qm->getSpaceInformation()->allocState();
      if(Qm->getC1()->getStateSpace()->getType() == ob::STATE_SPACE_SO3) {
        static_cast<ob::SO3StateSpace::StateType*>(s_C1)->setIdentity();
      }
      if(Qm->getC1()->getStateSpace()->getType() == ob::STATE_SPACE_SO2) {
        static_cast<ob::SO2StateSpace::StateType*>(s_C1)->setIdentity();
      }
      //Qm->SampleC1(s_C1);
      Qm->mergeStates(s_M0, s_C1, s_M1);
      quotientSpaces.at(m-1)->getSpaceInformation()->freeState(s_M0);
      Qm->getC1()->freeState(s_C1);
      s_M0 = s_M1;
    }
    v.setState(s_M0);
  }

}
