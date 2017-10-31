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
                                                           return ptc || S_1->hasSolution();
                                                       });
  auto cmp = [](SliceSpace* left, SliceSpace* right) 
              { 
                return left->GetSamplingDensity() < right->GetSamplingDensity();
              };
  std::priority_queue<SliceSpace*, std::vector<SliceSpace*>, decltype(cmp)> Q(cmp);

  Q.push(S_0);
  double growth_time = 1e-3;

  while(!ptcOrSolutionFound){
    SliceSpace* S = Q.top();
    S->Grow(growth_time);
    base::PathPtr path = S->GetShortestPath();
    if(path){

      //std::pair<edge_descr_type, bool> result = add_edge (u, v, props, g);

      //check if S is a horizontal or vertical space
      ///bool vertical = true;
      ///if(vertical){
      ///  //found solution in vertical space. this means we need to update the
      ///  //weight in S_0
      ///  //E = S->GetUnderlyingEdge()
      ///  //S_0->SetEdgeWeight(E)
      ///  //E->weight = d(s,t)
      ///}else{
      ///  //S_0!
      ///  //create new slicespace for an edge without slicespace
      ///  //iterator throuhg all edges
      ///  E = path->GetNextEdge()
      ///  if(E->S){
      ///    if(E->S->hasSolution()){
      ///      E->weight = d(E->s,E->t)
      ///    }else{
      ///      E->weight = +dInf;
      ///    }
      ///  }else{
      ///    //create new spaceinformationptr, using edge
      ///    E->S = new SliceSpace();
      ///    Q.push(E->S);
      ///    E->weight = +dInf;
      ///  }
      ///}
    }
  }
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

