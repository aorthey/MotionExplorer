#include "ompl_necessary.h"
#include <ompl/base/goals/GoalSampleableRegion.h>

using namespace ompl::geometric;
NecessaryRRT::NecessaryRRT(const base::SpaceInformationPtr &si, const base::SpaceInformationPtr &si2): RRT(si)
{
  setName("NecessaryRRT");
}
NecessaryRRT::~NecessaryRRT(void)
{
}
ob::PlannerStatus NecessaryRRT::solve(const ob::PlannerTerminationCondition &ptc)
{
    std::cout << "Planner " + getName() + " specs:" << std::endl;
    std::cout << "Multithreaded:                 " << (getSpecs().multithreaded ? "Yes" : "No") << std::endl;
    std::cout << "Reports approximate solutions: " << (getSpecs().approximateSolutions ? "Yes" : "No") << std::endl;
    std::cout << "Can optimize solutions:        " << (getSpecs().optimizingPaths ? "Yes" : "No") << std::endl;
    std::cout << "Aware of the following parameters:";
    std::vector<std::string> params;
    params_.getParamNames(params);
    for (auto &param : params)
        std::cout << " " << param;
    std::cout << std::endl;

    ///** \brief The space information for which planning is done */
    //SpaceInformationPtr si_;
    ///** \brief The user set problem definition */
    //ProblemDefinitionPtr pdef_;
    ///** \brief Utility class to extract valid input states  */
    //PlannerInputStates pis_;
    ///** \brief The name of this planner */
    //std::string name_;
    ///** \brief The specifications of the planner (its capabilities) */
    //PlannerSpecs specs_;
    ///** \brief A map from parameter names to parameter instances for this planner. This field is populated by
    // * the declareParam() function */
    //ParamSet params_;
    ///** \brief A mapping between this planner's progress property names and the functions used for querying
    // * those progress properties */
    //PlannerProgressProperties plannerProgressProperties_;
    ///** \brief Flag indicating whether setup() has been called */
    //bool setup_;
    checkValidity();
    ob::Goal *goal = pdef_->getGoal().get();
    auto *goal_s = dynamic_cast<ob::GoalSampleableRegion *>(goal);

    while (const ob::State *st = pis_.nextStart())
    {
        auto *motion = new Motion(si_);
        si_->copyState(motion->state, st);
        nn_->add(motion);
    }

    if (nn_->size() == 0)
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return ob::PlannerStatus::INVALID_START;
    }

    if (!sampler_)
        sampler_ = si_->allocStateSampler();

    OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(), nn_->size());

    Motion *solution = nullptr;
    Motion *approxsol = nullptr;
    double approxdif = std::numeric_limits<double>::infinity();
    auto *rmotion = new Motion(si_);
    ob::State *rstate = rmotion->state;
    ob::State *xstate = si_->allocState();

    while (!ptc)
    {
        /* sample random state (with goal biasing) */
        if ((goal_s != nullptr) && rng_.uniform01() < goalBias_ && goal_s->canSample())
            goal_s->sampleGoal(rstate);
        else
            sampler_->sampleUniform(rstate);

        /* find closest state in the tree */
        Motion *nmotion = nn_->nearest(rmotion);
        ob::State *dstate = rstate;

        /* find state to add */
        double d = si_->distance(nmotion->state, rstate);
        if (d > maxDistance_)
        {
            si_->getStateSpace()->interpolate(nmotion->state, rstate, maxDistance_ / d, xstate);
            dstate = xstate;
        }

        if (si_->checkMotion(nmotion->state, dstate))
        {
            /* create a motion */
            auto *motion = new Motion(si_);
            si_->copyState(motion->state, dstate);
            motion->parent = nmotion;

            nn_->add(motion);
            double dist = 0.0;
            bool sat = goal->isSatisfied(motion->state, &dist);
            if (sat)
            {
                approxdif = dist;
                solution = motion;
                break;
            }
            if (dist < approxdif)
            {
                approxdif = dist;
                approxsol = motion;
            }
        }
    }

    bool solved = false;
    bool approximate = false;
    if (solution == nullptr)
    {
        solution = approxsol;
        approximate = true;
    }

    if (solution != nullptr)
    {
        lastGoalMotion_ = solution;

        /* construct the solution path */
        std::vector<Motion *> mpath;
        while (solution != nullptr)
        {
            mpath.push_back(solution);
            solution = solution->parent;
        }

        /* set the solution path */
        auto path(std::make_shared<PathGeometric>(si_));
        for (int i = mpath.size() - 1; i >= 0; --i)
            path->append(mpath[i]->state);
        pdef_->addSolutionPath(path, approximate, approxdif, getName());
        solved = true;
    }

    si_->freeState(xstate);
    if (rmotion->state != nullptr)
        si_->freeState(rmotion->state);
    delete rmotion;

    OMPL_INFORM("%s: Created %u states", getName().c_str(), nn_->size());

    return ob::PlannerStatus(solved, approximate);

}
void NecessaryRRT::clear(void)
{
  RRT::clear();
}
void NecessaryRRT::setup(void)
{
  RRT::setup();
    //SelfConfig sc(si_, getName());
    //sc.configure...
}
void NecessaryRRT::getPlannerData(ob::PlannerData &data) const
{
  RRT::getPlannerData(data);
    // fill data with the states and edges that were created
    // in the exploration data structure
    // perhaps also fill control::PlannerData
}
