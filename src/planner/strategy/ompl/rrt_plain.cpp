#include "rrt_plain.h"

using namespace ompl::geometric;
RRTPlain::RRTPlain(const base::SpaceInformationPtr &si): RRT(si)
{
  setName("RRTPlain");
}
RRTPlain::~RRTPlain(void)
{
}
void RRTPlain::Sample(Configuration *q_random){
  if(rng_.uniform01() < goalBias_){
    goal->sampleGoal(q_random->state);
  }else{
    sampler_->sampleUniform(q_random->state);
  }
}
RRTPlain::Configuration* RRTPlain::Nearest(Configuration *q_random){
  return G_->nearest(q_random);
}
RRTPlain::Configuration* RRTPlain::Connect(Configuration *q_near, Configuration *q_random){
  //##############################################################################
  // q_new_state <- BALL_maxDistance_(q_near) in direction of q_random
  //##############################################################################
  double d = si_->distance(q_near->state, q_random->state);
  if(d > maxDistance_){
    si_->getStateSpace()->interpolate(q_near->state, q_random->state, maxDistance_ / d, q_random->state);
  }

  //##############################################################################
  // extend the tree from q_near towards q_new
  //##############################################################################
  if(si_->checkMotion(q_near->state, q_random->state)){
    auto *q_new = new Configuration(si_);
    si_->copyState(q_new->state, q_random->state);
    q_new->parent = q_near;
    G_->add(q_new);
    return q_new;
  }
  return nullptr;
}
bool RRTPlain::ConnectedToGoal(Configuration* q){
  double dist=0.0;
  if(q != nullptr){
    if(goal->isSatisfied(q->state, &dist)){
      return true;
    }
  }
  return false;
}
void RRTPlain::ConstructSolution(Configuration *q_goal){
  if (q_goal != nullptr){

    std::vector<Configuration *> q_path;
    while (q_goal != nullptr){
      q_path.push_back(q_goal);
      q_goal = q_goal->parent;
    }

    auto path(std::make_shared<PathGeometric>(si_));
    for (int i = q_path.size() - 1; i >= 0; --i){
      path->append(q_path[i]->state);
    }
    pdef_->addSolutionPath(path);
  }
}
void RRTPlain::Init(){
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

  checkValidity();
  goal = dynamic_cast<ob::GoalSampleableRegion *>(pdef_->getGoal().get());

  while (const ob::State *st = pis_.nextStart()){
    auto *q_start = new Configuration(si_);
    si_->copyState(q_start->state, st);
    G_->add(q_start);
  }
  if (G_->size() == 0){
    OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
    exit(0);
  }
  if (!sampler_) sampler_ = si_->allocStateSampler();
}


void RRTPlain::clear()
{
  Planner::clear();
  sampler_.reset();
  freeMemory();
  if(G_){
    G_->clear();
  }
}

void RRTPlain::setup(void)
{
  Planner::setup();
  tools::SelfConfig sc(si_, getName());
  sc.configurePlannerRange(maxDistance_);

  if (!G_){
    G_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Configuration *>(this));
  }
  G_->setDistanceFunction([this](const Configuration *a, const Configuration *b)
                           {
                              return si_->distance(a->state, b->state);
                           });

}

void RRTPlain::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    std::vector<Configuration *> vertices;

    if (G_){
      G_->list(vertices);
    }

    if (lastExtendedConfiguration != nullptr){
      data.addGoalVertex(base::PlannerDataVertex(lastExtendedConfiguration->state));
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

void RRTPlain::Grow(){
  //Grow
  Configuration *q_random = new Configuration(si_);
  Configuration *q_near = nullptr;
  Configuration *q_new = nullptr;

  Sample(q_random);
  q_near = Nearest(q_random);
  q_new = Connect(q_near, q_random);

  if(q_new != nullptr){
    lastExtendedConfiguration = q_new;
  }
  if(q_random->state != nullptr){
    si_->freeState(q_random->state);
  }
  delete q_random;
}


ob::PlannerStatus RRTPlain::solve(const ob::PlannerTerminationCondition &ptc)
{
  Init();

  while(!ptc){

    Grow();

    //CheckSolution
    if(ConnectedToGoal(lastExtendedConfiguration)){
      ConstructSolution(lastExtendedConfiguration);
      return ob::PlannerStatus::EXACT_SOLUTION;
    }
  }

  OMPL_INFORM("%s: Created %u states", getName().c_str(), G_->size());
  return ob::PlannerStatus::TIMEOUT;
}

