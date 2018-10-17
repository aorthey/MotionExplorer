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
  root_chart = new T(si_vec.at(0), nullptr);
  root_chart->SetLevel(0);

  for(uint k = 0; k < si_vec.size(); k++){
    og::QuotientChart* parent = nullptr;
    if(k>0) parent = quotientCharts.back();
    T* chart_k = new T(si_vec.at(k), parent);
    quotientCharts.push_back(chart_k);
  }

  //for(uint k = 0; k < si_vec.size(); k++){
  //  og::QuotientChart* parent = nullptr;
  //  if(k>0) parent = quotientCharts.back();
  //  T* ss = new T(si_vec.at(k), parent);
  //  quotientCharts.push_back(ss);
  //}

  //have a copy of each quotient-space, needed to cast down states at the end
}

template <class T>
MultiChart<T>::~MultiChart(){
}

template <class T>
void MultiChart<T>::setup(){
  if(!setup_) BaseT::setup();
  if(pdef_){
    std::cout << "SETUP MULTICHART" << std::endl;

    root_chart->setProblemDefinition(pdef_vec.at(0));
    root_chart->setup();

    found_path_on_last_level = false;
    saturated_levels = false;

  }else{
    OMPL_INFORM("%s: problem definition is not set, deferring setup completion...", getName().c_str());
    setup_ = false;
  }
}

template <class T>
void MultiChart<T>::clear(){
  BaseT::clear();
  std::cout << "CLEAR MULTICHART" << std::endl;

  found_path_on_last_level = false;
  saturated_levels = false;

  solutions.clear();
  pdef_->clearSolutionPaths();
  for(uint k = 0; k < pdef_vec.size(); k++){
    pdef_vec.at(k)->clearSolutionPaths();
  }


  if(root_chart) root_chart->clear();
  root_chart->setProblemDefinition(pdef_vec.at(0));
  dynamic_cast<QuotientChart*>(root_chart)->DeleteSubCharts();

  std::cout << si_vec.at(0)->getStateValidityChecker()->getSpecs().hasValidDirectionComputation << std::endl;

  std::cout << "FINISHED CLEARING MULTICHART" << std::endl;
  //for(uint k = 0; k < quotientCharts.size(); k++){
  //  quotientCharts.at(k)->clear();
  //}
}


template <class T>
ob::PlannerStatus MultiChart<T>::solve(const base::PlannerTerminationCondition &ptc)
{
  static const double T_GROW = 0.01; //time to grow before Checking if solution exists

  auto cmp = [](og::QuotientChart* left, og::QuotientChart* right) 
              { 
                return left->GetImportance() < right->GetImportance();
              };

  std::priority_queue<og::QuotientChart*, std::vector<og::QuotientChart*>, decltype(cmp)> Q(cmp);

  base::PlannerTerminationCondition ptcOrSolutionFound([this, &ptc]
                                 { return ptc || found_path_on_last_level || saturated_levels; });
  Q.push(root_chart);

  //the leaves contain the direction in which we like to expand the tree,
  //non-leave nodes are only used for refinement of a specific leave, iff they happen to lie along the path (inside the
  //tree) from the root_chart node towards the particular leave.
  //Put another way, if we grow a leave node quotient chart, then all its
  //parents will be refined (in that way we can make sure that the algorithm is
  //complete)
  //Qleaves.push_back(root_chart);

  og::QuotientChart* current_chart = root_chart;

  //each chart has an associated importance weight. If we operate on a certain
  //path [i,j,k], then the charts Q_i, Q_ij and Q_ijk have the usual importance
  //as in the quotientspace approach. All other charts are set to zero.

  ompl::time::point t_start = ompl::time::now();
  while(!ptcOrSolutionFound)
  {
    //growing the charts occurs in the path below current_chart.
    og::QuotientChart* jChart = Q.top();
    Q.pop();
    jChart->Grow(T_GROW);

    if(current_chart->IsSaturated()){
      //level is saturated, if there are no more charts to consider there is
      //nothing to be done
      if(Q.empty()) saturated_levels = true;
    }else{
      if(current_chart->FoundNewPath()){
        uint k = current_chart->GetLevel();
        double t_k_end = ompl::time::seconds(ompl::time::now() - t_start);

        std::cout << std::string(80, '#') << std::endl;
        std::cout << "Found Path on Level " << k+1 << "/" << levels << " after " << t_k_end << " seconds." << std::endl;
        std::cout << std::string(80, '-') << std::endl;
        std::cout << *current_chart << std::endl;
        std::cout << std::string(80, '#') << std::endl;

        if(k == levels-1){
          found_path_on_last_level = true;

          //add solution
          base::PathPtr sol;
          current_chart->GetSolution(sol);
          base::PlannerSolution psol(sol);
          psol.setPlannerName(getName());
          pdef_->addSolutionPath(psol);

          Q.push(jChart);
        }else{
          //not yet reached maximal level. create a new sibling chart (containing
          //the solution path plus associated vertices), and create
          //a child of the sibling (the nullspace of the solution path plus its
          //associated vertices on the next quotientchart).
          // std::cout << "found path on level " << k << " (vertices: " << current_chart->GetNumberOfVertices() << ")" << std::endl;
          // std::cout << "Trying to extract path subgraph" << std::endl;
          //#####################################################################
          //Local Chart (containing path plus neighborhood)
          //#####################################################################

          og::QuotientChart *local = new T(si_vec.at(k), dynamic_cast<T*>(current_chart->GetParent()));
          local->setProblemDefinition(pdef_vec.at(k));
          local->CopyChartFromSibling(current_chart, current_chart->GetChartNumberOfComponents()-1);
          local->SetLevel(k);
          local->SetChartHorizontalIndex(current_chart->GetChartNumberOfComponents());

          current_chart->AddChartSibling(local);

          //#####################################################################
          //Global Chart (contains the whole quotient pointed to by the local chart)
          //#####################################################################
          og::QuotientChart *global = new T(si_vec.at(k+1), local);
          global->setProblemDefinition(pdef_vec.at(k+1));
          global->setup();
          global->SetLevel(k+1);

          //DFS approach: forget about the current node, just take its sibling and
          //work on that pathway. We can backtrack later by going through the
          //parent/sibling pointers
          while( !Q.empty() ) Q.pop();

          og::QuotientChart *levels_to_be_added = global;
          //while(levels_to_be_added!=nullptr)
          //{
          //  Q.push(levels_to_be_added);
          //  levels_to_be_added = dynamic_cast<og::QuotientChart*>(levels_to_be_added->GetParent());
          //}
          Q.push(levels_to_be_added);

          //Qleaves.push_back(global);
          //note that Q contains only the latest added node, while Qleaves
          //contains all. 
          current_chart = global;


        }
      }else{
        Q.push(jChart);
      }
    }
  }

  while(!Q.empty()) Q.pop();
  return (found_path_on_last_level? ob::PlannerStatus::EXACT_SOLUTION : ob::PlannerStatus::TIMEOUT);
}


template <class T>
void MultiChart<T>::setProblemDefinition(std::vector<ob::ProblemDefinitionPtr> &pdef_)
{
  pdef_vec = pdef_;
  ob::Planner::setProblemDefinition(pdef_vec.back());
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
  QuotientChart *current = root_chart;
  current->getPlannerData(data);

  for(uint vidx = 0; vidx < data.numVertices(); vidx++)
  {
    PlannerDataVertexAnnotated &v = *static_cast<PlannerDataVertexAnnotated*>(&data.getVertex(vidx));
    v.SetMaxLevel(levels);
    uint k = v.GetLevel();

    // std::cout << data.vertexIndex(v) << " vertex " << vidx << "/" << data.numVertices() << std::endl;
    // std::cout << v.GetOpenNeighborhoodDistance() << std::endl;
    // si_vec.at(k)->printState(v.getState());
    const ob::State *s_V = v.getState();
    ob::State *s_Q0 = si_vec.at(k)->cloneState(s_V);

    for(uint m = k+1; m < levels; m++){
      og::QuotientChart *Qm = quotientCharts.at(m);
      ob::State *s_X1 = Qm->GetX1()->allocState();
      ob::State *s_Q1 = Qm->getSpaceInformation()->allocState();
      if(Qm->GetX1()->getStateSpace()->getType() == ob::STATE_SPACE_SO3) {
        static_cast<ob::SO3StateSpace::StateType*>(s_X1)->setIdentity();
      }
      if(Qm->GetX1()->getStateSpace()->getType() == ob::STATE_SPACE_SO2) {
        static_cast<ob::SO2StateSpace::StateType*>(s_X1)->setIdentity();
      }
      Qm->MergeStates(s_Q0, s_X1, s_Q1);
      quotientCharts.at(m-1)->getSpaceInformation()->freeState(s_Q0);
      Qm->GetX1()->freeState(s_X1);
      s_Q0 = s_Q1;
    }
    v.setState(s_Q0);
  }

}
