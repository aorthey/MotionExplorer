#include "ompl_necessary.h"

using namespace ompl;
NecessaryRRT::NecessaryRRT(const base::SpaceInformationPtr &si) : base::Planner(si, "NecessaryRRT")
{
    //specs_.approximateSolutions = ...;
    //specs_.recognizedGoal = ...;
}
NecessaryRRT::~NecessaryRRT(void)
{
}
base::PlannerStatus NecessaryRRT::solve(const base::PlannerTerminationCondition &ptc)
{
    std::cout << "[" << getName() << "] planning" << std::endl;
    checkValidity();
    // get a handle to the Goal from the ompl::base::ProblemDefinition member, pdef_
    base::Goal *goal = pdef_->getGoal().get();
    // get input states with PlannerInputStates helper, pis_
    while (const base::State *st = pis_.nextStart())
    {
        // st will contain a start state.  Typically this state will
        // be cloned here and inserted into the Planner's data structure.
    }
    // if needed, sample states from the goal region (and wait until a state is sampled)
    const base::State *st = pis_.nextGoal(ptc);
    // or sample a new goal state only if available:
    //const base::State *st = pis_.nextGoal();
    // periodically check if ptc() returns true.
    // if it does, terminate planning.
    while (ptc() == false)
    {
        // Start planning here.
        // call routines from SpaceInformation (si_) as needed. i.e.,
        // si_->allocStateSampler() for sampling,
        // si_->checkMotion(state1, state2) for state validity, etc...
        // use the Goal pointer to evaluate whether a sampled state satisfies the goal requirements
        // use log macros for informative messaging, i.e., logInfo("Planner found a solution!");
    }
    // When a solution path is computed, save it here
    //pdef_->addSolutionPath(...);
    // Return a value from the PlannerStatus enumeration.
    // See ompl::base::PlannerStatus for the possible return values
    return base::PlannerStatus::EXACT_SOLUTION;
}
void NecessaryRRT::clear(void)
{
    Planner::clear();
}
void NecessaryRRT::setup(void)
{
    Planner::setup();
    //SelfConfig sc(si_, getName());
    //sc.configure...
}
void NecessaryRRT::getPlannerData(base::PlannerData &data) const
{
    // fill data with the states and edges that were created
    // in the exploration data structure
    // perhaps also fill control::PlannerData
}
