#include "slicespace_prm.h"
#include "GoalVisitor.hpp"
#include "planner/validitychecker/validity_checker_ompl.h"

#include <ompl/geometric/planners/prm/ConnectionStrategy.h>
#include <ompl/base/goals/GoalSampleableRegion.h>
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

SliceSpacePRM::SliceSpacePRM(RobotWorld *world_, const base::SpaceInformationPtr &si0, const ob::SpaceInformationPtr &si1)
  : base::Planner(si0, "SliceSpacePRM")
  , S_0(new SliceSpace(si1))
  , S_1(new SliceSpace(si0))
  , world(world_)
  , si_level0(si1)
  , si_level1(si0)
{
  worldsettings.InitializeDefault(*world);

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
  Planner::setup();
  S_0->setProblemDefinition(pdef_);
  S_0->setup();
}

void SliceSpacePRM::setProblemDefinition(const base::ProblemDefinitionPtr &pdef)
{
  Planner::setProblemDefinition(pdef);
  S_0->setProblemDefinition(pdef);
}


void SliceSpacePRM::clear()
{
  Planner::clear();
  S_0->clear();
  //S_1->clear();
}


ompl::base::PlannerStatus SliceSpacePRM::solve(const base::PlannerTerminationCondition &ptc)
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

  while(!ptcOrSolutionFound){
    SliceSpace* S = Q.top();
    Q.pop();
    std::cout << "grow: ("<< S->id << ")" << slice_growth_time << std::endl;
    S->Grow(slice_growth_time);
    base::PathPtr path = S->GetSolutionPath();
    if(path){
      std::cout << "found sol" << std::endl;
      if(S->horizontal){
        //S_0
        std::vector<SliceSpace::Vertex> vpath = S->VerticesAlongShortestPath();
        for(uint k = 0; k < vpath.size()-1; k++){
          //SliceSpace::EdgeProperty& e = S->GetEdgeAlongShortestPath(k);
          //std::cout << "edge with cost " << e.getCost() << std::endl;
          SliceSpace::Vertex v = vpath.at(k);
          SliceSpace::Vertex w = vpath.at(k+1);

          std::pair<SliceSpace::Edge, bool> edge = boost::edge(v, w, S->graph);
          SliceSpace::EdgeProperty e = get (boost::edge_weight_t(), S->graph, edge.first);

          if(!e.slicespace){
            std::cout << "new slicespace" << std::endl;
            //create new slicespace
            e.slicespace = CreateNewSliceSpaceEdge(v,w,0);
            e.setWeight(+dInf);
            boost::put(boost::edge_weight_t(), S->graph, edge.first, e);
            Q.push(e.slicespace);
            break;
          }else{
            if(e.slicespace->hasSolution()){
              //make sure that we update the edge
              e.setOriginalWeight();
              boost::put(boost::edge_weight_t(), S->graph, edge.first, e);
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
      }
    }
    Q.push(S);
  }

  std::cout << "S_1->GetSolutionPath()" << std::endl;
  //base::PathPtr sol = S_1->GetSolutionPath();

  //deconstruct priority queue
  while(!Q.empty()){
    SliceSpace *S = Q.top();
    ob::SpaceInformationPtr si = S_1->getSpaceInformation();
    if(!S->horizontal){
      //add slicespace data points to S_1
      std::cout << "extracting data from SliceSpace #"<< S->id << std::endl;

      SliceSpace::Vertex v1 = S->GetExternalAssociatedEdgeSource();
      SliceSpace::Vertex v2 = S->GetExternalAssociatedEdgeTarget();

      //std::pair<SliceSpace::Edge, bool> edge = boost::edge(v, w, S_0->graph);
      //SliceSpace::EdgeProperty e = get(boost::edge_weight_t(), S_0->graph, edge.first);
      //const SliceSpace::Vertex v1 = boost::source(edge.first, S_0->graph);
      //const SliceSpace::Vertex v2 = boost::target(edge.first, S_0->graph);

      //std::cout << v1 << std::endl;
      //std::cout << v2 << std::endl;

      ob::State *s0 = S_0->stateProperty_[v1];
      ob::State *s1 = S_0->stateProperty_[v2];
      const ob::RealVectorStateSpace::StateType *R1 = s0->as<ob::RealVectorStateSpace::StateType>();
      const ob::RealVectorStateSpace::StateType *R2 = s1->as<ob::RealVectorStateSpace::StateType>();

      //for all vertices
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

        SO2 = s0->as<ob::CompoundState>()->as<ob::SO2StateSpace::StateType>(1);
        q1(3) = SO2->value;
        SO2 = s1->as<ob::CompoundState>()->as<ob::SO2StateSpace::StateType>(1);
        q2(3) = SO2->value;

        double t = s0->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[0];
        for(uint k = 0; k < 2; k++)
          q1(k) = R1->values[k] + t*(R2->values[k]-R1->values[k]);

        t = s1->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[0];
        for(uint k = 0; k < 2; k++)
          q2(k) = R1->values[k] + t*(R2->values[k]-R1->values[k]);


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

                //stateProperty_[m] = si_->cloneState(workStates[i]);

        SliceSpace::EdgeProperty properties(ob::Cost(0.0));//S_1->opt_->motionCost(S_1->stateProperty_[m], S_1->stateProperty_[n]));
        boost::add_edge(m, n, properties, S_1->graph);
      }

      //std::pair<SliceSpace::Edge, bool> edge = boost::edge(v, w, S_0->graph);
      //SliceSpace::EdgeProperty e = get(boost::edge_weight_t(), S_0->graph, edge.first);
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
  //if (sol)
  //{
  //  base::PlannerSolution psol(sol);
  //  psol.setPlannerName(getName());
  //  //psol.setOptimized(opt_, bestCost_, S_0->hasSolution());
  //  pdef_->addSolutionPath(psol);
  //}

  return sol ? base::PlannerStatus::EXACT_SOLUTION : base::PlannerStatus::TIMEOUT;
}

void SliceSpacePRM::getPlannerData(base::PlannerData &data) const
{
  Planner::getPlannerData(data);
  //S_0->getPlannerData(data);
  S_1->getPlannerData(data);
}

SliceSpace* SliceSpacePRM::CreateNewSliceSpaceEdge(SliceSpace::Vertex v, SliceSpace::Vertex w, double yaw){

  //WorldPlannerSettings worldsettings;

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
  //edge_cspace->print();

  ob::SpaceInformationPtr si_edge = edge_cspace->SpaceInformationPtr();

  Config qi; qi.resize(robot->q.size()); qi.setZero();
  qi(0) = q1(0);
  qi(1) = q1(1);
  qi(3) = yaw;

  Config qg; qg.resize(robot->q.size()); qg.setZero();
  qg(0) = q2(0);
  qg(1) = q2(1);
  qg(3) = 0;

  SliceSpace* S = new SliceSpace(si_edge);
  S->horizontal = false;
  S->SetExternalAssociatedEdgeSource(v);
  S->SetExternalAssociatedEdgeTarget(w);

  ob::ProblemDefinitionPtr pdef = std::make_shared<ob::ProblemDefinition>(si_edge);
  pdef->addStartState(edge_cspace->ConfigToOMPLState(qi));

  auto goal=std::make_shared<GoalStateFinalEdge>(si_edge);
  goal->setState(edge_cspace->ConfigToOMPLState(qg));
  goal->setThreshold(0.001);
  pdef->setGoal(goal);
  S->setProblemDefinition(pdef);

  // DEBUG
  //ob::State* s = pdef->getStartState(0);
  //si_edge->printState(s, std::cout);
  //std::cout << edge_cspace->OMPLStateToConfig(s) << std::endl;
  //std::cout << si_edge->getStateValidityChecker()->isValid(s) << std::endl;
  //exit(0);

  S->setup();

  return S;
}
