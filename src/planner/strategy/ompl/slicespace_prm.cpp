#include "slicespace_prm.h"
#include "GoalVisitor.hpp"
#include "planner/validitychecker/validity_checker_ompl.h"

#include <ompl/geometric/planners/prm/ConnectionStrategy.h>
#include <ompl/base/goals/GoalSampleableRegion.h>
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
  , S_0(new SliceSpace(si0))
  , S_1(new SliceSpace(si1))
  , world(world_)
  , si_level0(si0)
  , si_level1(si1)
{
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
  S_1->clear();
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
              //edge is OK, go to next edge
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

        SliceSpace::Vertex& v = S->GetExternalAssociatedEdgeSource();
        SliceSpace::Vertex& w = S->GetExternalAssociatedEdgeTarget();

        std::pair<SliceSpace::Edge, bool> edge = boost::edge(v, w, S_0->graph);
        SliceSpace::EdgeProperty e = get(boost::edge_weight_t(), S_0->graph, edge.first);
        e.setOriginalWeight();
        boost::put(boost::edge_weight_t(), S_0->graph, edge.first, e);
      }
    }
  }
  std::cout << "timeout!" << std::endl;
  base::PathPtr sol = S_0->GetSolutionPath();

  if (sol)
  {
      base::PlannerSolution psol(sol);
      psol.setPlannerName(getName());
      //psol.setOptimized(opt_, bestCost_, S_0->hasSolution());
      pdef_->addSolutionPath(psol);
  }

  return sol ? base::PlannerStatus::EXACT_SOLUTION : base::PlannerStatus::TIMEOUT;
}

void SliceSpacePRM::getPlannerData(base::PlannerData &data) const
{
  Planner::getPlannerData(data);
  S_0->getPlannerData(data);
}

SliceSpace* SliceSpacePRM::CreateNewSliceSpaceEdge(SliceSpace::Vertex v, SliceSpace::Vertex w, double yaw){

  WorldPlannerSettings worldsettings;
  worldsettings.InitializeDefault(*world);

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
  ob::SpaceInformationPtr si_edge = std::make_shared<ob::SpaceInformation>(edge_cspace->SpacePtr());

  Config qi; qi.resize(robot->q.size()); qi.setZero();
  qi(0) = q1(0);
  qi(1) = q1(1);
  qi(3) = yaw;

  ///GET position from q1

  SliceSpace* S = new SliceSpace(si_edge);

  ob::ProblemDefinitionPtr pdef = std::make_shared<ob::ProblemDefinition>(si_edge);
  pdef->addStartState(edge_cspace->ConfigToOMPLState(qi));
  ob::GoalPtr goal = std::make_shared<GoalRegionEdge>(si_edge);
  pdef->setGoal(goal);
  S->setProblemDefinition(pdef);

  ob::State* s = pdef->getStartState(0);
  std::cout << s << std::endl;
  exit(0);

  S->setup();

  return S;
}
