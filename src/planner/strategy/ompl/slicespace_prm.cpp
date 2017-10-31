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
                                                           return ptc || S_0->hasSolution();
                                                       });
  //auto cmp = [](SliceSpace left, SliceSpace right) 
  //            { 
  //              return true;
  //            };
  //std::priority_queue<SliceSpace*, std::vector<SliceSpace*>, cmp > Q;

  //S_0->solve(ptcOrSolutionFound);

  while(!ptcOrSolutionFound){
    //SliceSpace* S = Q.pop();
    S_0->Grow();
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

