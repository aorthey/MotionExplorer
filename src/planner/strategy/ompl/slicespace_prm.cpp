#include "slicespace_prm.h"
#include "GoalVisitor.hpp"

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

SliceSpacePRM::SliceSpacePRM(const base::SpaceInformationPtr &si0, const ob::SpaceInformationPtr &si1)
  : base::Planner(si0, "SliceSpacePRM")
  , S_0(new SliceSpace(si0))
  , S_1(new SliceSpace(si1))
  , si_level0(si0)
  , si_level1(si1)
{
  S_0->horizontal = true;
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
                return left->GetSamplingDensity() < right->GetSamplingDensity();
              };
  std::priority_queue<SliceSpace*, std::vector<SliceSpace*>, decltype(cmp)> Q(cmp);

  S_0->horizontal = true;
  Q.push(S_0);
  double slice_growth_time = 1e-3;

  while(!ptcOrSolutionFound){
    SliceSpace* S = Q.top();
    S->Grow(slice_growth_time);
    base::PathPtr path = S->GetSolutionPath();
    std::cout << "grow: " << slice_growth_time << std::endl;
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

          //Vertex v1 = source(edge.first,graph);
          //Vertex v2 = target(edge.first,graph);
          if(!e.slicespace){
            std::cout << "new slicespace" << std::endl;
            //create new slicespace
            e.slicespace = CreateNewSliceSpaceEdge();
            e.setWeight(+dInf);
            boost::put(boost::edge_weight_t(), S->graph, edge.first, e);
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

#include "planner/cspace/cspace_factory.h"
SliceSpace* SliceSpacePRM::CreateNewSliceSpaceEdge(){

 // int robot_idx = input->robot_idx;
 // Robot *robot = world->robots[robot_idx];
 // SingleRobotCSpace *kcspace = new SingleRobotCSpace(*world,robot_idx,&worldsettings);
 // CSpaceOMPL *cspace = factory.MakeGeometricCSpaceR2(robot, kcspace);

  //SliceSpace* S =  new SliceSpace(si_level1);
  return NULL;
}
