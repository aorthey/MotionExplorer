#include "slicespace_prm.h"
#include "GoalVisitor.hpp"
#include "planner/validitychecker/validity_checker_ompl.h"

#include <ompl/geometric/planners/prm/ConnectionStrategy.h>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/datastructures/PDF.h>
#include <ompl/tools/config/SelfConfig.h>
#include <ompl/tools/config/MagicConstants.h>
#include <boost/graph/astar_search.hpp>
#include <boost/graph/incremental_components.hpp>
#include <boost/property_map/vector_property_map.hpp>
#include <boost/foreach.hpp>
#include <queue>
#include <functional>

#define foreach BOOST_FOREACH

using namespace og;
static ob::OptimizationObjectivePtr getThresholdPathLengthObj2(const ob::SpaceInformationPtr& si)
{
  ob::OptimizationObjectivePtr obj(new ob::PathLengthOptimizationObjective(si));
  obj->setCostThreshold(ob::Cost(dInf));
  return obj;
}

SliceSpacePRM::SliceSpacePRM(RobotWorld *world_, const base::SpaceInformationPtr &si0, const ob::SpaceInformationPtr &si1)
  : base::Planner(si1, "SliceSpacePRM")
  , S_0(new SliceSpace(si0))
  , S_1(new SliceSpace(si1))
  , world(world_)
  , si_level0(si0)
  , si_level1(si1)
{
  worldsettings.InitializeDefault(*world);

  //##############################################

  S_0->horizontal = true;
  S_1->horizontal = true;

  specs_.recognizedGoal = base::GOAL_SAMPLEABLE_REGION;
  specs_.approximateSolutions = false;
  specs_.optimizingPaths = true;
}

SliceSpacePRM::~SliceSpacePRM()
{
  clear();
}

void SliceSpacePRM::setup()
{
  S_0->setup();
  S_1->setup();
}

void SliceSpacePRM::setProblemDefinition(const base::ProblemDefinitionPtr &pdef)
{
  S_1->setProblemDefinition(pdef);
  exit(0);
}
void SliceSpacePRM::setProblemDefinitionLevel0(const base::ProblemDefinitionPtr &pdef)
{
  S_0->setProblemDefinition(pdef);
}
void SliceSpacePRM::setProblemDefinitionLevel1(const base::ProblemDefinitionPtr &pdef)
{
  S_1->setProblemDefinition(pdef);
}


void SliceSpacePRM::clear()
{
  S_0->clear();
  S_1->clear();
}


ob::PlannerStatus SliceSpacePRM::solve(const base::PlannerTerminationCondition &ptc)
{
  base::PlannerTerminationCondition ptcOrSolutionFound([this, &ptc]
                                                       {
                                                           //return ptc || S_1->hasSolution();
                                                           return ptc;
                                                       });
  auto cmp = [](SliceSpace* left, SliceSpace* right) 
              { 
                return left->GetSamplingDensity() > right->GetSamplingDensity();
              };

  std::priority_queue<SliceSpace*, std::vector<SliceSpace*>, decltype(cmp)> Q(cmp);

  S_0->horizontal = true;
  Q.push(S_0);
  double slice_growth_time = 1e-3;

  bool solutionFound = false;
  while(!ptcOrSolutionFound && !solutionFound){
    SliceSpace* S = Q.top();
    Q.pop();
    std::cout << "grow: ("<< S->id << ")" << slice_growth_time << std::endl;
    S->Grow(slice_growth_time);
    base::PathPtr path = S->GetSolutionPath();
    if(S->hasSolution()){
      std::cout << "found sol at SliceSpace #" << S->id << std::endl;
      if(S->horizontal){
        //S_0
        std::vector<SliceSpace::Vertex> vpath = S->VerticesAlongShortestPath();
        ob::State *start = S_1->getProblemDefinition()->getStartState(0);
        ob::SO2StateSpace::StateType *SO2;
        SO2 = start->as<ob::CompoundState>()->as<ob::SO2StateSpace::StateType>(1);
        double yaw = SO2->value;
        std::vector<ob::State*> spath;
        spath.push_back(start);
        for(uint k = 0; k < vpath.size()-1; k++){
          SliceSpace::Vertex v = vpath.at(k);
          SliceSpace::Vertex w = vpath.at(k+1);

          std::pair<SliceSpace::Edge, bool> edge = boost::edge(v, w, S->graph);
          SliceSpace::EdgeProperty e = get (boost::edge_weight_t(), S->graph, edge.first);

          const SliceSpace::Vertex v1 = boost::source(edge.first, S->graph);
          const SliceSpace::Vertex v2 = boost::target(edge.first, S->graph);

          if(e.slicespace==NULL){
            if(k==vpath.size()-2){
              std::cout << "Final Edge!" << std::endl;
              const ob::GoalPtr gp = S_1->getProblemDefinition()->getGoal();
              ob::State *goal = static_pointer_cast<ob::GoalState>(gp)->getState();
              SO2 = goal->as<ob::CompoundState>()->as<ob::SO2StateSpace::StateType>(1);
              double goal_yaw = SO2->value;
              e.slicespace = CreateNewSliceSpaceFinalGoalEdge(v1, v2, yaw, goal_yaw);
            }else{
              e.slicespace = CreateNewSliceSpaceEdge(v1,v2,yaw);
            }
            e.setWeight(+dInf);
            boost::put(boost::edge_weight_t(), S->graph, edge.first, e);
            Q.push(e.slicespace);
            break;
          }else{
            if(e.slicespace->hasSolution()){
              //make sure that we update the edge
              e.setOriginalWeight();
              boost::put(boost::edge_weight_t(), S->graph, edge.first, e);

              SliceSpace *Sedge = e.slicespace;
              base::PathPtr path = Sedge->GetSolutionPath();

              //how to extract yaw?

              og::PathGeometric gpath = static_cast<og::PathGeometric&>(*path);
              std::vector<ob::State *> states = gpath.getStates();
              ob::State *st_final = states.back();
              SO2 = st_final->as<ob::CompoundState>()->as<ob::SO2StateSpace::StateType>(1);
              yaw = SO2->value;

              if(k==vpath.size()-2){
                solutionFound = true;
              }

            }else{
              e.setWeight(+dInf);
              boost::put(boost::edge_weight_t(), S->graph, edge.first, e);
              break;
            }
          }
        }
      }else{
        //S_edge has solution: we need to update the associated edge in the S_0
        //graph!

        const SliceSpace::Vertex v = S->GetExternalAssociatedEdgeSource();
        const SliceSpace::Vertex w = S->GetExternalAssociatedEdgeTarget();

        std::pair<SliceSpace::Edge, bool> edge = boost::edge(v, w, S_0->graph);
        SliceSpace::EdgeProperty e = get(boost::edge_weight_t(), S_0->graph, edge.first);
        e.setOriginalWeight();
        boost::put(boost::edge_weight_t(), S_0->graph, edge.first, e);
        //S->heuristic_add = +dInf;
      }
    }
    Q.push(S);
  }

  //construct shortest path

  if(solutionFound){
    std::vector<SliceSpace::Vertex> sp = S_0->VerticesAlongShortestPath();
    std::cout << "constructing solution. vertices = " << sp.size() << std::endl;
    for(uint k = 0; k < sp.size(); k++){
      std::cout << sp.at(k) << std::endl;
    }

    SliceSpace::Vertex v0_0 = sp.at(0);
    SliceSpace::Vertex v0_1 = S_1->startM_.at(0);

    ob::RealVectorStateSpace::StateType *R;
    ob::SO2StateSpace::StateType *SO2;

    for(uint k = 1; k < sp.size(); k++){
      SliceSpace::Vertex v1_0 = sp.at(k);
      std::cout << "edge : " << v0_0 << " <-> " << v1_0 << std::endl;

      std::pair<SliceSpace::Edge, bool> edge = boost::edge(v0_0, v1_0, S_0->graph);
      if(!edge.second){
        std::cout << "edge does not exists between " << v0_0 << " and " << v1_0 << std::endl;
        exit(0);
      }
      SliceSpace::EdgeProperty pk = get (boost::edge_weight_t(), S_0->graph, edge.first);
      if(!pk.slicespace){
        std::cout << "has no slicespace" << std::endl;
        exit(0);
      }
      SliceSpace *S = pk.slicespace;

      ///Get S_0 coordinates
      ob::State *s0_0 = S_0->stateProperty_[v0_0];
      ob::State *s1_0 = S_0->stateProperty_[v1_0];
      const ob::RealVectorStateSpace::StateType *R1 = s0_0->as<ob::RealVectorStateSpace::StateType>();
      const ob::RealVectorStateSpace::StateType *R2 = s1_0->as<ob::RealVectorStateSpace::StateType>();

      ///Get S_edge coordinates [0,1] \times C_2
      base::PathPtr path = S->GetShortestPath();
      og::PathGeometric gpath = static_cast<og::PathGeometric&>(*path);
      std::vector<ob::State *> states = gpath.getStates();

      std::cout << "  slice vertices: " << states.size() << std::endl;
      for(uint j = 1; j < states.size(); j++){

        ob::State* s0_e = states.at(j-1);

        R = s0_e->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0);
        SO2 = s0_e->as<ob::CompoundState>()->as<ob::SO2StateSpace::StateType>(1);
        double t0 = R->values[0];
        double yaw0 = SO2->value;

        ob::State* s1_e = states.at(j);

        R = s1_e->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0);
        SO2 = s1_e->as<ob::CompoundState>()->as<ob::SO2StateSpace::StateType>(1);
        double t1 = R->values[0];
        double yaw1 = SO2->value;

        /// Create new S_1 state at s0
        ob::ScopedState<> state0(S_1->getSpaceInformation());
        R = state0->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0);
        SO2 = state0->as<ob::CompoundState>()->as<ob::SO2StateSpace::StateType>(1);
        for(uint i = 0; i < 2; i++)
          R->values[i] = R1->values[i] + t0*(R2->values[i]-R1->values[i]);
        SO2->value = yaw0;

        /// Create new S_1 state at s1
        ob::ScopedState<> state1(S_1->getSpaceInformation());
        R = state1->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0);
        SO2 = state1->as<ob::CompoundState>()->as<ob::SO2StateSpace::StateType>(1);
        for(uint i = 0; i < 2; i++)
          R->values[i] = R1->values[i] + t1*(R2->values[i]-R1->values[i]);
        SO2->value = yaw1;

        /// Add states s0,s1 and edge (s0,s1) to the S_1 roadmap
        SliceSpace::Vertex v1_1;
        if(k>sp.size()-2 && j>states.size()-2){
          v1_1 = S_1->goalM_.at(0);
        }else{
          v1_1 = boost::add_vertex(S_1->graph);
        }

        S_1->stateProperty_[v0_1] = si_->cloneState(state0.get());
        S_1->stateProperty_[v1_1] = si_->cloneState(state1.get());
        SliceSpace::EdgeProperty properties(S_1->opt_->motionCost(S_1->stateProperty_[v0_1], S_1->stateProperty_[v1_1]));
        boost::add_edge(v0_1, v1_1, properties, S_1->graph);

        v0_1 = v1_1;
      }
      v0_0 = v1_0;

//      ob::State* s1_e = states.back();
//
//      R = s0_e->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0);
//      SO2 = s0_e->as<ob::CompoundState>()->as<ob::SO2StateSpace::StateType>(1);
//      double t0 = R->values[0];
//      double yaw0 = SO2->value;
//
//      R = s1_e->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0);
//      SO2 = s1_e->as<ob::CompoundState>()->as<ob::SO2StateSpace::StateType>(1);
//      double t1 = R->values[0];
//      double yaw1 = SO2->value;
//
//      /// Create new S_1 state at s0
//      ob::ScopedState<> state0(S_1->getSpaceInformation());
//      R = state0->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0);
//      SO2 = state0->as<ob::CompoundState>()->as<ob::SO2StateSpace::StateType>(1);
//      for(uint k = 0; k < 2; k++)
//        R->values[k] = R1->values[k] + t0*(R2->values[k]-R1->values[k]);
//      SO2->value = yaw0;
//
//      /// Create new S_1 state at s1
//      ob::ScopedState<> state1(S_1->getSpaceInformation());
//      R = state1->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0);
//      SO2 = state1->as<ob::CompoundState>()->as<ob::SO2StateSpace::StateType>(1);
//      for(uint k = 0; k < 2; k++)
//        R->values[k] = R1->values[k] + t1*(R2->values[k]-R1->values[k]);
//      SO2->value = yaw1;
//
//      /// Add states s0,s1 and edge (s0,s1) to the S_1 roadmap
//      SliceSpace::Vertex v1_1;
//      if(k>sp.size()-2){
//        v1_1 = S_1->goalM_.at(0);
//      }else{
//        v1_1 = boost::add_vertex(S_1->graph);
//      }
//
//      S_1->stateProperty_[v0_1] = si_->cloneState(state0.get());
//      S_1->stateProperty_[v1_1] = si_->cloneState(state1.get());
//      SliceSpace::EdgeProperty properties(S_1->opt_->motionCost(S_1->stateProperty_[v0_1], S_1->stateProperty_[v1_1]));
//      boost::add_edge(v0_1, v1_1, properties, S_1->graph);
//
//      v0_0 = v1_0;
//      v0_1 = v1_1;

      ///for(uint j = 1; j < states.size(); j++){
      ///  ob::State *second = states.at(j);
      ///  R = second->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0);
      ///  SO2 = second->as<ob::CompoundState>()->as<ob::SO2StateSpace::StateType>(1);
      ///  double t2 = R->values[0];
      ///  double yaw2 = SO2->value;

      ///  ///Merge coordinates
      ///  ob::ScopedState<> stateN(S_1->getSpaceInformation());
      ///  R = stateN->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0);
      ///  SO2 = stateN->as<ob::CompoundState>()->as<ob::SO2StateSpace::StateType>(1);
      ///  for(uint k = 0; k < 2; k++)
      ///    R->values[k] = R1->values[k] + t2*(R2->values[k]-R1->values[k]);
      ///  SO2->value = yaw2;

      ///  SliceSpace::Vertex n = boost::add_vertex(S_1->graph);
      ///  S_1->stateProperty_[n] = si_->cloneState(stateN.get());

      ///  //SliceSpace::EdgeProperty properties(ob::Cost(0.0));//S_1->opt_->motionCost(S_1->stateProperty_[m], S_1->stateProperty_[n]));
      ///  SliceSpace::EdgeProperty properties(S_1->opt_->motionCost(S_1->stateProperty_[m], S_1->stateProperty_[n]));
      ///  boost::add_edge(m, n, properties, S_1->graph);

      ///  stateM = stateN;
      ///  m = n;
      ///}
    }
  }




  //deconstruct priority queue
  std::cout << std::string(80, '-') << std::endl;
  while(!Q.empty()){
    SliceSpace *S = Q.top();
    ob::SpaceInformationPtr si = S_1->getSpaceInformation();
    std::cout << "extracting data from SliceSpace #"<< S->id << " (" 
      << boost::num_vertices(S->graph) <<  " vertices, " 
      << boost::num_edges(S->graph) << " edges, " 
      << S->GetSamplingDensity() << " density, " << S->getSpaceInformation()->getSpaceMeasure() << " measure)" << std::endl;
    if(!S->horizontal){
      //add slicespace data points to S_1

      SliceSpace::Vertex v1 = S->GetExternalAssociatedEdgeSource();
      SliceSpace::Vertex v2 = S->GetExternalAssociatedEdgeTarget();

      ob::State *s0 = S_0->stateProperty_[v1];
      ob::State *s1 = S_0->stateProperty_[v2];
      const ob::RealVectorStateSpace::StateType *R1 = s0->as<ob::RealVectorStateSpace::StateType>();
      const ob::RealVectorStateSpace::StateType *R2 = s1->as<ob::RealVectorStateSpace::StateType>();

      uint N = boost::num_vertices(S->graph);
      ob::RealVectorStateSpace::StateType *R;
      ob::SO2StateSpace::StateType *SO2;
      foreach (const SliceSpace::Edge e, boost::edges(S->graph)){

        const SliceSpace::Vertex v1 = boost::source(e, S->graph);
        const SliceSpace::Vertex v2 = boost::target(e, S->graph);
        ob::State *s0 = S->stateProperty_[v1];
        ob::State *s1 = S->stateProperty_[v2];

        Config q1;q1.resize(6);q1.setZero();
        Config q2;q2.resize(6);q2.setZero();

        double t = s0->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[0];
        for(uint k = 0; k < 2; k++)
          q1(k) = R1->values[k] + t*(R2->values[k]-R1->values[k]);
        SO2 = s0->as<ob::CompoundState>()->as<ob::SO2StateSpace::StateType>(1);
        q1(3) = SO2->value;

        t = s1->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[0];
        for(uint k = 0; k < 2; k++)
          q2(k) = R1->values[k] + t*(R2->values[k]-R1->values[k]);
        SO2 = s1->as<ob::CompoundState>()->as<ob::SO2StateSpace::StateType>(1);
        q2(3) = SO2->value;

        ob::ScopedState<> stateM(si);
        R = stateM->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0);
        SO2 = stateM->as<ob::CompoundState>()->as<ob::SO2StateSpace::StateType>(1);
        R->values[0] = q1(0);
        R->values[1] = q1(1);
        SO2->value = q1(3);
        ob::ScopedState<> stateN(si);
        R = stateN->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0);
        SO2 = stateN->as<ob::CompoundState>()->as<ob::SO2StateSpace::StateType>(1);
        R->values[0] = q2(0);
        R->values[1] = q2(1);
        SO2->value = q2(3);

        SliceSpace::Vertex m = boost::add_vertex(S_1->graph);
        SliceSpace::Vertex n = boost::add_vertex(S_1->graph);
        S_1->stateProperty_[m] = si_->cloneState(stateM.get());
        S_1->stateProperty_[n] = si_->cloneState(stateN.get());

        SliceSpace::EdgeProperty properties(ob::Cost(0.0));//S_1->opt_->motionCost(S_1->stateProperty_[m], S_1->stateProperty_[n]));
        boost::add_edge(m, n, properties, S_1->graph);
      }

    }else{
      //horizontal

      foreach (const SliceSpace::Edge e, boost::edges(S->graph))
      {
        const SliceSpace::Vertex v1 = boost::source(e, S->graph);
        const SliceSpace::Vertex v2 = boost::target(e, S->graph);
        ob::State *s0 = S->stateProperty_[v1];
        ob::State *s1 = S->stateProperty_[v2];
        const ob::RealVectorStateSpace::StateType *R1 = s0->as<ob::RealVectorStateSpace::StateType>();
        const ob::RealVectorStateSpace::StateType *R2 = s1->as<ob::RealVectorStateSpace::StateType>();

        SliceSpace::Vertex m = boost::add_vertex(S_1->graph);
        SliceSpace::Vertex m2 = boost::add_vertex(S_1->graph);
        SliceSpace::Vertex n = boost::add_vertex(S_1->graph);
        SliceSpace::Vertex n2 = boost::add_vertex(S_1->graph);

        ob::RealVectorStateSpace::StateType *R;
        ob::SO2StateSpace::StateType *SO2;

        ob::ScopedState<> stateM(si);
        R = stateM->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0);
        SO2 = stateM->as<ob::CompoundState>()->as<ob::SO2StateSpace::StateType>(1);
        R->values[0] = R1->values[0];
        R->values[1] = R1->values[1];
        SO2->value = 0;
        ob::ScopedState<> stateN(si);
        R = stateN->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0);
        SO2 = stateN->as<ob::CompoundState>()->as<ob::SO2StateSpace::StateType>(1);
        R->values[0] = R2->values[0];
        R->values[1] = R2->values[1];
        SO2->value = 0;

        ob::ScopedState<> stateM2(si);
        R = stateM2->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0);
        SO2 = stateM2->as<ob::CompoundState>()->as<ob::SO2StateSpace::StateType>(1);
        R->values[0] = R1->values[0];
        R->values[1] = R1->values[1];
        SO2->value = 2*M_PI;

        ob::ScopedState<> stateN2(si);
        R = stateN2->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0);
        SO2 = stateN2->as<ob::CompoundState>()->as<ob::SO2StateSpace::StateType>(1);
        R->values[0] = R2->values[0];
        R->values[1] = R2->values[1];
        SO2->value = 2*M_PI;

        S_1->stateProperty_[m] = si_->cloneState(stateM.get());
        S_1->stateProperty_[m2] = si_->cloneState(stateM2.get());
        S_1->stateProperty_[n] = si_->cloneState(stateN.get());
        S_1->stateProperty_[n2] = si_->cloneState(stateN2.get());

        SliceSpace::EdgeProperty properties(ob::Cost(0.0));//S_1->opt_->motionCost(S_1->stateProperty_[m], S_1->stateProperty_[n]));
        boost::add_edge(m, n, properties, S_1->graph);
        //boost::add_edge(m, m2, properties, S_1->graph);
        //boost::add_edge(n, n2, properties, S_1->graph);
        //boost::add_edge(m2, n2, properties, S_1->graph);
      }
    }

    Q.pop();
  }

  base::PathPtr sol = NULL;
  if(solutionFound){
    std::cout << "S_1->GetSolutionPath()" << std::endl;
    sol = S_1->GetSolutionPath();
    if (sol)
    {
      std::cout << "found solution" << std::endl;
      base::PlannerSolution psol(sol);
      psol.setPlannerName(getName());
      //psol.setOptimized(opt_, bestCost_, S_0->hasSolution());
      ob::ProblemDefinitionPtr pdef = S_0->getProblemDefinition();
      S_1->getProblemDefinition()->addSolutionPath(psol);
    }
  }

  return sol ? ob::PlannerStatus::EXACT_SOLUTION : ob::PlannerStatus::TIMEOUT;
}

void SliceSpacePRM::getPlannerData(base::PlannerData &data) const
{
  S_1->getPlannerData(data);
}

SliceSpace* SliceSpacePRM::CreateNewSliceSpaceEdge(SliceSpace::Vertex v, SliceSpace::Vertex w, double yaw){

  CSpaceInput cin;
  cin.timestep_max = 0.1;
  cin.timestep_min = 0.01;
  CSpaceFactory factory(cin);

  int robot_idx = 0;
  Robot *robot = world->robots[robot_idx];
  SingleRobotCSpace *kcspace = new SingleRobotCSpace(*world,robot_idx,&worldsettings);

  ob::SpaceInformationPtr si = S_0->getSpaceInformation();
  const ob::StateValidityCheckerPtr checker = si->getStateValidityChecker();
  OMPLValidityCheckerPtr ochecker = static_pointer_cast<OMPLValidityChecker>(checker);
  CSpaceOMPL *cspace = ochecker->ompl_space;
  ob::State* sv = S_0->stateProperty_[v];
  Config q1 = cspace->OMPLStateToConfig(sv);
  ob::State* sw = S_0->stateProperty_[w];
  Config q2 = cspace->OMPLStateToConfig(sw);

  CSpaceOMPL *edge_cspace = factory.MakeGeometricCSpaceR2EdgeSO2(robot, kcspace, q1, q2);

  ob::SpaceInformationPtr si_edge = edge_cspace->SpaceInformationPtr();

  Config qi; qi.resize(robot->q.size()); qi.setZero();
  qi(0) = q1(0);
  qi(1) = q1(1);
  qi(3) = yaw;

  Config qg; qg.resize(robot->q.size()); qg.setZero();
  qg(0) = q2(0);
  qg(1) = q2(1);
  qg(3) = yaw;

  SliceSpace* S = new SliceSpace(si_edge);
  S->horizontal = false;
  S->SetExternalAssociatedEdgeSource(v);
  S->SetExternalAssociatedEdgeTarget(w);

  ob::ProblemDefinitionPtr pdef = std::make_shared<ob::ProblemDefinition>(si_edge);
  pdef->addStartState(edge_cspace->ConfigToOMPLState(qi));

  auto goal=std::make_shared<GoalStateFinalEdge>(si_edge);
  goal->setState(edge_cspace->ConfigToOMPLState(qg));
  goal->setThreshold(0.01);
  pdef->setGoal(goal);
  pdef->setOptimizationObjective( getThresholdPathLengthObj2(si_edge) );
  S->setProblemDefinition(pdef);
  S->setup();

  return S;
}
SliceSpace* SliceSpacePRM::CreateNewSliceSpaceFinalGoalEdge(SliceSpace::Vertex v, SliceSpace::Vertex w, double yaw, double goal_yaw){

  CSpaceInput cin;
  cin.timestep_max = 0.1;
  cin.timestep_min = 0.01;
  CSpaceFactory factory(cin);

  int robot_idx = 0;
  Robot *robot = world->robots[robot_idx];
  SingleRobotCSpace *kcspace = new SingleRobotCSpace(*world,robot_idx,&worldsettings);

  ob::SpaceInformationPtr si = S_0->getSpaceInformation();
  const ob::StateValidityCheckerPtr checker = si->getStateValidityChecker();
  OMPLValidityCheckerPtr ochecker = static_pointer_cast<OMPLValidityChecker>(checker);
  CSpaceOMPL *cspace = ochecker->ompl_space;
  ob::State* sv = S_0->stateProperty_[v];
  Config q1 = cspace->OMPLStateToConfig(sv);
  ob::State* sw = S_0->stateProperty_[w];
  Config q2 = cspace->OMPLStateToConfig(sw);

  CSpaceOMPL *edge_cspace = factory.MakeGeometricCSpaceR2EdgeSO2(robot, kcspace, q1, q2);

  ob::SpaceInformationPtr si_edge = edge_cspace->SpaceInformationPtr();

  Config qi; qi.resize(robot->q.size()); qi.setZero();
  qi(0) = q1(0);
  qi(1) = q1(1);
  qi(3) = yaw;

  Config qg; qg.resize(robot->q.size()); qg.setZero();
  qg(0) = q2(0);
  qg(1) = q2(1);
  qg(3) = goal_yaw;

  SliceSpace* S = new SliceSpace(si_edge);
  S->horizontal = false;
  S->SetExternalAssociatedEdgeSource(v);
  S->SetExternalAssociatedEdgeTarget(w);

  ob::ProblemDefinitionPtr pdef = std::make_shared<ob::ProblemDefinition>(si_edge);
  pdef->addStartState(edge_cspace->ConfigToOMPLState(qi));

  auto goal=std::make_shared<ob::GoalState>(si_edge);
  goal->setState(edge_cspace->ConfigToOMPLState(qg));
  goal->setThreshold(0.01);
  pdef->setGoal(goal);
  pdef->setOptimizationObjective( getThresholdPathLengthObj2(si_edge) );
  S->setProblemDefinition(pdef);
  S->setup();

  return S;
}
