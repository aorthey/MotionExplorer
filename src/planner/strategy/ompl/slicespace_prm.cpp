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
  , roadmap(new SliceSpace(si0))
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
}

void SliceSpacePRM::setProblemDefinition(const base::ProblemDefinitionPtr &pdef)
{
  Planner::setProblemDefinition(pdef);
}


void SliceSpacePRM::clear()
{
  Planner::clear();
  roadmap->clear();
}


ompl::base::PlannerStatus SliceSpacePRM::solve(const base::PlannerTerminationCondition &ptc)
{

    base::PlannerTerminationCondition ptcOrSolutionFound([this, &ptc]
                                                         {
                                                             return ptc || roadmap->hasSolution();
                                                         });

    while(!ptcOrSolutionFound()){
      roadmap->Grow();
    }
    base::PathPtr sol = roadmap->GetSolutionPath();

    if (sol)
    {
        base::PlannerSolution psol(sol);
        psol.setPlannerName(getName());
        //psol.setOptimized(opt_, bestCost_, roadmap->hasSolution());
        pdef_->addSolutionPath(psol);
    }

    return sol ? base::PlannerStatus::EXACT_SOLUTION : base::PlannerStatus::TIMEOUT;
}

void SliceSpacePRM::getPlannerData(base::PlannerData &data) const
{
  Planner::getPlannerData(data);
  roadmap->getPlannerData(data);
}

