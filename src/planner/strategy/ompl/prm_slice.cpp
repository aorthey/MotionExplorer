#include "prm_slice.h"
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

namespace ompl
{
  namespace magic
  {
    static const unsigned int MAX_RANDOM_BOUNCE_STEPS = 5;
    static const double ROADMAP_BUILD_TIME = 0.01;
    static const unsigned int DEFAULT_NEAREST_NEIGHBORS = 10;
  }
}
PRMSlice::PRMSlice(const ob::SpaceInformationPtr &si)
  : og::PRMBasic(si)
{
  if (!isSetup())
    setup();
  if (!sampler_){
    sampler_ = si_->allocValidStateSampler();
  }
  if (!simpleSampler_){
    simpleSampler_ = si_->allocStateSampler();
  }

  xstates.resize(magic::MAX_RANDOM_BOUNCE_STEPS);
  si_->allocStates(xstates);

  std::cout << "Hello this is PRMSlice with measure=" << si_->getSpaceMeasure() << std::endl;
}

PRMSlice::~PRMSlice(){
  si_->freeStates(xstates);
}

ob::PlannerStatus PRMSlice::Init(){
  checkValidity();
  auto *goal = dynamic_cast<ob::GoalSampleableRegion *>(pdef_->getGoal().get());

  if (goal == nullptr){
    OMPL_ERROR("%s: Unknown type of goal", getName().c_str());
    exit(0);
  }

  while (const ob::State *st = pis_.nextStart()){
    startM_.push_back(addMilestone(si_->cloneState(st)));
  }
  if (startM_.empty()){
    OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
    exit(0);
  }
  if (!goal->couldSample()){
    OMPL_ERROR("%s: Insufficient states in sampleable goal region", getName().c_str());
    exit(0);
  }

  if (goal->maxSampleCount() > goalM_.size() || goalM_.empty()){
    const ob::State *st = pis_.nextGoal();
    if (st != nullptr){
      goalM_.push_back(addMilestone(si_->cloneState(st)));
    }
  }
  unsigned long int nrStartStates = boost::num_vertices(g_);
  OMPL_INFORM("%s: Starting planning with %lu states already in datastructure", getName().c_str(), nrStartStates);
}

ob::PlannerStatus PRMSlice::solve(const ob::PlannerTerminationCondition &ptc){
  Init();

  addedNewSolution_ = false;
  base::PathPtr sol;

  bestCost_ = opt_->infiniteCost();

  base::PlannerTerminationCondition ptcOrSolutionFound([this, &ptc]
                                 { return ptc || addedNewSolution_; });

  while (!ptcOrSolutionFound())
  {
    Grow(magic::ROADMAP_BUILD_TIME);
    checkForSolution(sol);
  }

  OMPL_INFORM("%s: Created %u states", getName().c_str(), boost::num_vertices(g_));

  if (sol)
  {
    base::PlannerSolution psol(sol);
    psol.setPlannerName(getName());
    psol.setOptimized(opt_, bestCost_, addedNewSolution_);
    pdef_->addSolutionPath(psol);
  }

  return sol ? base::PlannerStatus::EXACT_SOLUTION : base::PlannerStatus::TIMEOUT;
}

void PRMSlice::Grow(double t){
  double twothird = (2.0/3.0)*t;
  double onethird = (1.0/3.0)*t;
  growRoadmap(ob::timedPlannerTerminationCondition(twothird), xstates[0]);
  expandRoadmap( ob::timedPlannerTerminationCondition(onethird), xstates);
}
void PRMSlice::checkForSolution(ob::PathPtr &solution)
{
  bool foundSolution = maybeConstructSolution(startM_, goalM_, solution);
  if(foundSolution && !addedNewSolution_){
    addedNewSolution_ = true;
  }
}
bool PRMSlice::hasSolution(){
  return addedNewSolution_;
}
void PRMSlice::growRoadmap(const ob::PlannerTerminationCondition &ptc, ob::State *workState)
{
  while (!ptc)
  {
    iterations_++;
    bool found = false;
    while (!found && !ptc)
    {
      unsigned int attempts = 0;
      do
      {
        found = Sample(workState);
        attempts++;
      } while (attempts < magic::FIND_VALID_STATE_ATTEMPTS_WITHOUT_TERMINATION_CHECK && !found);
    }
    if (found) addMilestone(si_->cloneState(workState));
  }
}

bool PRMSlice::Sample(ob::State *workState){
  return sampler_->sample(workState);
}



