#include "ompl_necessary.h"
#include <ompl/base/goals/GoalSampleableRegion.h>

using namespace ompl::geometric;
NecessaryRRT::NecessaryRRT(const base::SpaceInformationPtr &si, const base::SpaceInformationPtr &si2): RRT(si), si_level2(si2)
{
  setName("NecessaryRRT");
}
NecessaryRRT::~NecessaryRRT(void)
{
}
ob::PlannerStatus NecessaryRRT::solve(const ob::PlannerTerminationCondition &ptc)
{
    //setRange(0.1);
  
    std::cout << "Planner " + getName() + " specs:" << std::endl;
    std::cout << "Multithreaded:                 " << (getSpecs().multithreaded ? "Yes" : "No") << std::endl;
    std::cout << "Reports approximate solutions: " << (getSpecs().approximateSolutions ? "Yes" : "No") << std::endl;
    std::cout << "Can optimize solutions:        " << (getSpecs().optimizingPaths ? "Yes" : "No") << std::endl;
    std::cout << "Range:                         " << getRange() << std::endl;
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

    //let us extend Configuration (the tree) by adding to each node its edge to the
    //parent. That means we add a new RRT problem between parent and edge, but
    //this time inside the si2 space, constrained by the edge of course. How
    //would we do that? maybe make it the complement space, i.e. si -> R2, and
    //si2 -> S1 ?
    //have a second sampler, in this sampler we only sample the complement space
    //plus we sample [0,1] along the edge. then we need to be able to connect to
    //a valid 0 sample and a valid 1 sample! How about first sampling those
    //spaces? i.e. 0 x S1 and 1 x S1 , and then once we found some, we can grow
    //on [0,1] x S1?  Well we can actually start from the start configuration,
    //and grow towards the goal region which is 1 x S1 (as we did before). Then
    //once we found a valid path, we store it, and we put it into the edge. If
    //we come back later from another edge, and get a configuration on 1xS1,
    //then we need to be able to connect them.

    // only check the shortest path edges. read KH15 to see if that is similar

    checkValidity();
    ob::Goal *goal = pdef_->getGoal().get();
    auto *goal_s = dynamic_cast<ob::GoalSampleableRegion *>(goal);

    while (const ob::State *st = pis_.nextStart()){
      auto *q_start = new Configuration(si_);
      si_->copyState(q_start->state, st);
      G_->add(q_start);
    }

    if (G_->size() == 0){
      OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
      return ob::PlannerStatus::INVALID_START;
    }

    if (!sampler_) sampler_ = si_->allocStateSampler();

    Configuration *solution = nullptr;
    Configuration *approxsol = nullptr;
    double approxdif = std::numeric_limits<double>::infinity();

    //class Configuration
    //Configuration(const base::SpaceInformationPtr &si) : state(si->allocState())
    //base::State *state{nullptr};
    //Configuration *parent{nullptr};
    //
    //NearestNeighbor<Configuration*> G_ //equals a priority queue

    auto *q_random = new Configuration(si_);
    ob::State *q_random_state = q_random->state;
    ob::State *xstate = si_->allocState();

    while(!ptc){
      //##############################################################################
      // q_random <- SAMPLE() + goal biasing
      //##############################################################################
      if((goal_s != nullptr) && rng_.uniform01() < goalBias_ && goal_s->canSample()){
          goal_s->sampleGoal(q_random_state);
      }else{
          sampler_->sampleUniform(q_random_state);
      }

      //##############################################################################
      // q_near <- NEAREST(G_, q_random)
      //##############################################################################
      Configuration *q_near = G_->nearest(q_random);
      ob::State *q_near_state = q_near->state;

      //##############################################################################
      // q_new_state <- BALL_maxDistance_(q_near) in direction of q_random
      //##############################################################################
      ob::State *q_new_state = q_random_state;
      double d = si_->distance(q_near_state, q_random_state);
      if(d > maxDistance_){
        si_->getStateSpace()->interpolate(q_near_state, q_random_state, maxDistance_ / d, xstate);
        q_new_state = xstate;
      }

      //##############################################################################
      // extend the tree from q_near towards q_new
      //##############################################################################
      if(si_->checkMotion(q_near_state, q_new_state)){
        auto *q_new = new Configuration(si_);
        si_->copyState(q_new->state, q_new_state);
        q_new->parent = q_near;
        G_->add(q_new);

        double dist = 0.0;
        bool satisfied = goal->isSatisfied(q_new->state, &dist);
        if(satisfied){
          //check edges, grow vertically
          approxdif = dist;
          solution = q_new;
          break;
        }
        if(dist < approxdif){
          approxdif = dist;
          approxsol = q_new;
        }
      }
    }

    bool solved = false;
    bool approximate = false;
    if (solution == nullptr){
      solution = approxsol;
      approximate = true;
    }

    if (solution != nullptr){
      lastGoalConfiguration_ = solution;

      std::vector<Configuration *> q_path;
      while (solution != nullptr){
        q_path.push_back(solution);
        solution = solution->parent;
      }

      auto path(std::make_shared<PathGeometric>(si_));
      for (int i = q_path.size() - 1; i >= 0; --i){
        path->append(q_path[i]->state);
      }
      pdef_->addSolutionPath(path, approximate, approxdif, getName());
      solved = true;
    }

    si_->freeState(xstate);
    if (q_random->state != nullptr){
      si_->freeState(q_random->state);
    }
    delete q_random;

    OMPL_INFORM("%s: Created %u states", getName().c_str(), G_->size());

    return ob::PlannerStatus(solved, approximate);

}
void NecessaryRRT::clear()
{
  Planner::clear();
  sampler_.reset();
  freeMemory();
  if(G_){
    G_->clear();
  }
  lastGoalConfiguration_ = nullptr;
}

void NecessaryRRT::setup(void)
{
  Planner::setup();
  tools::SelfConfig sc(si_, getName());
  sc.configurePlannerRange(maxDistance_);

  if (!G_){
    G_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Configuration *>(this));
  }
  G_->setDistanceFunction([this](const Configuration *a, const Configuration *b)
                           {
                               return distanceFunction(a, b);
                           });

}

void NecessaryRRT::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    std::vector<Configuration *> vertices;

    if (G_){
      G_->list(vertices);
    }

    if (lastGoalConfiguration_ != nullptr){
      data.addGoalVertex(base::PlannerDataVertex(lastGoalConfiguration_->state));
    }

    for (auto &vertex : vertices)
    {
      if (vertex->parent == nullptr){
        data.addStartVertex(base::PlannerDataVertex(vertex->state));
      }else{
        data.addEdge(base::PlannerDataVertex(vertex->parent->state), base::PlannerDataVertex(vertex->state));
      }
    }
}

