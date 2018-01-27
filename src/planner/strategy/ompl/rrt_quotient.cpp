#include "rrt_quotient.h"
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/tools/config/SelfConfig.h>

using namespace ompl::geometric;
using namespace ompl::base;
using namespace og;

RRTQuotient::RRTQuotient(const base::SpaceInformationPtr &si, Quotient *previous_) : 
  Quotient(si, previous_)
{
  setName("RRTQuotient");
  specs_.recognizedGoal = base::GOAL_SAMPLEABLE_REGION;
  specs_.directed = true;
  Planner::declareParam<double>("range", this, &RRTQuotient::setRange, &RRTQuotient::getRange, "0.:1.:10000.");
  connectionPoint_ = std::make_pair<base::State *, base::State *>(nullptr, nullptr);
  startTree = true;
}

RRTQuotient::~RRTQuotient()
{
    freeMemory();
}

void RRTQuotient::setup()
{
  Planner::setup();
  tools::SelfConfig sc(si_, getName());
  sc.configurePlannerRange(maxDistance_);

  if (!tStart_)
      tStart_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Configuration *>(this));
  if (!tGoal_)
      tGoal_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Configuration *>(this));
  tStart_->setDistanceFunction([this](const Configuration *a, const Configuration *b)
                               {
                                   return distanceFunction(a, b);
                               });
  tGoal_->setDistanceFunction([this](const Configuration *a, const Configuration *b)
                              {
                                  return distanceFunction(a, b);
                              });
}
void RRTQuotient::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    std::vector<Configuration *> configurations;
    if (tStart_)
        tStart_->list(configurations);

    for (auto &configuration : configurations)
    {
        if (configuration->parent == nullptr)
            data.addStartVertex(base::PlannerDataVertex(configuration->state, 1));
        else
        {
            data.addEdge(base::PlannerDataVertex(configuration->parent->state, 1), base::PlannerDataVertex(configuration->state, 1));
        }
    }

    configurations.clear();
    if (tGoal_)
        tGoal_->list(configurations);

    for (auto &configuration : configurations)
    {
        if (configuration->parent == nullptr)
            data.addGoalVertex(base::PlannerDataVertex(configuration->state, 2));
        else
        {
            // The edges in the goal tree are reversed to be consistent with start tree
            data.addEdge(base::PlannerDataVertex(configuration->state, 2), base::PlannerDataVertex(configuration->parent->state, 2));
        }
    }

    // Add the edge connecting the two trees
    data.addEdge(data.vertexIndex(connectionPoint_.first), data.vertexIndex(connectionPoint_.second));
}


void RRTQuotient::freeMemory()
{
    std::vector<Configuration *> configurations;

    if (tStart_)
    {
        tStart_->list(configurations);
        for (auto &configuration : configurations)
        {
            if (configuration->state != nullptr)
                si_->freeState(configuration->state);
            delete configuration;
        }
    }

    if (tGoal_)
    {
        tGoal_->list(configurations);
        for (auto &configuration : configurations)
        {
            if (configuration->state != nullptr)
                si_->freeState(configuration->state);
            delete configuration;
        }
    }
}

void RRTQuotient::clear()
{
  Planner::clear();
  sampler_.reset();
  freeMemory();
  if(tStart_){
    tStart_->clear();
  }
  if(tGoal_){
    tGoal_->clear();
  }
  connectionPoint_ = std::make_pair<base::State *, base::State *>(nullptr, nullptr);
  pis_.restart();
  startConfiguration = nullptr;
  goalConfiguration = nullptr;
  isTreeConnected = false;
  isSolved = false;
}

RRTQuotient::GrowState RRTQuotient::growTree(TreeData &tree, TreeGrowingInfo &tgi,
                                                                             Configuration *rconfiguration)
{
    /* find closest state in the tree */
    Configuration *nconfiguration = tree->nearest(rconfiguration);

    /* assume we can reach the state we go towards */
    bool reach = true;

    /* find state to add */
    base::State *dstate = rconfiguration->state;
    double d = si_->distance(nconfiguration->state, rconfiguration->state);
    if (d > maxDistance_)
    {
        si_->getStateSpace()->interpolate(nconfiguration->state, rconfiguration->state, maxDistance_ / d, tgi.xstate);
        dstate = tgi.xstate;
        reach = false;
    }
    // if we are in the start tree, we just check the configuration like we normally do;
    // if we are in the goal tree, we need to check the configuration in reverse, but checkMotion() assumes the first state it
    // receives as argument is valid,
    // so we check that one first
    bool validConfiguration = tgi.start ?
                           si_->checkMotion(nconfiguration->state, dstate) :
                           si_->getStateValidityChecker()->isValid(dstate) && si_->checkMotion(dstate, nconfiguration->state);

    if (validConfiguration)
    {
      auto *configuration = new Configuration(si_);
      si_->copyState(configuration->state, dstate);
      configuration->parent = nconfiguration;
      configuration->root = nconfiguration->root;
      configuration->parent_edge_weight = distanceFunction(configuration, configuration->parent);
      tgi.xconfiguration = configuration;

      tree->add(configuration);
      return reach ? REACHED : ADVANCED;
    }
    else
      return TRAPPED;
}
void RRTQuotient::Init(){

  //std::cout << "Planner " + getName() + " specs:" << std::endl;
  //std::cout << "Multithreaded:                 " << (getSpecs().multithreaded ? "Yes" : "No") << std::endl;
  //std::cout << "Reports approximate solutions: " << (getSpecs().approximateSolutions ? "Yes" : "No") << std::endl;
  //std::cout << "Can optimize solutions:        " << (getSpecs().optimizingPaths ? "Yes" : "No") << std::endl;
  //std::cout << "Range:                         " << getRange() << std::endl;
  //std::cout << "Aware of the following parameters:";
  //std::vector<std::string> params;
  //params_.getParamNames(params);
  //for (auto &param : params)
  //    std::cout << " " << param;
  //std::cout << std::endl;

  checkValidity();
  goal = dynamic_cast<ob::GoalSampleableRegion *>(pdef_->getGoal().get());
  if (goal == nullptr)
  {
      OMPL_ERROR("%s: Unknown type of goal", getName().c_str());
      exit(0);
  }

  //add start and goal state to their respective trees
  while (const ob::State *st = pis_.nextStart()){
    auto *q_start = new Configuration(si_);
    si_->copyState(q_start->state, st);
    q_start->root = q_start->state;
    tStart_->add(q_start);
  }
  if (tStart_->size() == 0){
    OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
    exit(0);
  }
  while (const ob::State *st = pis_.nextGoal()){
    auto *q_goal = new Configuration(si_);
    si_->copyState(q_goal->state, st);
    q_goal->root = q_goal->state;
    tGoal_->add(q_goal);
  }
  if (tGoal_->size() == 0){
    OMPL_ERROR("%s: There are no valid goal states!", getName().c_str());
    exit(0);
  }
  if (!sampler_) sampler_ = si_->allocStateSampler();

  tgi.xstate = si_->allocState();
}

ob::PathPtr RRTQuotient::ConstructSolution(Configuration *q_start, Configuration *q_goal){
  if (q_start->parent != nullptr){
      q_start = q_start->parent;
  }else{
      q_goal = q_goal->parent;
  }

  connectionPoint_ = std::make_pair(q_start->state, q_goal->state);

  /* construct the solution path */
  Configuration *solution = q_start;
  std::vector<Configuration *> mpath1;
  while (solution != nullptr)
  {
    mpath1.push_back(solution);
    solution = solution->parent;
  }

  solution = q_goal;
  std::vector<Configuration *> mpath2;
  while (solution != nullptr)
  {
    mpath2.push_back(solution);
    solution = solution->parent;
  }

  auto path(std::make_shared<PathGeometric>(si_));
  path->getStates().reserve(mpath1.size() + mpath2.size());
  for (int i = mpath1.size() - 1; i >= 0; --i)
    path->append(mpath1[i]->state);
  for (auto &i : mpath2)
    path->append(i->state);

  pdef_->addSolutionPath(path, false, 0.0, getName());
  return path;
}



void RRTQuotient::Grow(double t)
{
  TreeData &tree = startTree ? tStart_ : tGoal_;
  tgi.start = startTree;
  startTree = !startTree;
  TreeData &otherTree = startTree ? tStart_ : tGoal_;

  Configuration *q_random = new Configuration(si_);
  q_random->state = M1->allocState();
  Sample(q_random->state);

  GrowState gs = growTree(tree, tgi, q_random);
  if (!isTreeConnected && gs != TRAPPED)
  {
    Configuration *addedConfiguration = tgi.xconfiguration;

    if (gs != REACHED){
      si_->copyState(q_random->state, tgi.xstate);
    }

    GrowState gsc = ADVANCED;
    tgi.start = startTree;
    while (gsc == ADVANCED)
    {
      gsc = growTree(otherTree, tgi, q_random);
    }

    startConfiguration = startTree ? tgi.xconfiguration : addedConfiguration;
    goalConfiguration = startTree ? addedConfiguration : tgi.xconfiguration;

    if(gsc == REACHED){
      isTreeConnected = true;
    }

  }
  si_->freeState(q_random->state);
  delete q_random;
}
bool RRTQuotient::ConnectedToGoal(){
  return isTreeConnected && goal->isStartGoalPairValid(startConfiguration->root, goalConfiguration->root);
}

void RRTQuotient::CheckForSolution(ob::PathPtr &solution)
{
  if(ConnectedToGoal()){
    solution = ConstructSolution(startConfiguration, goalConfiguration);
    isSolved = true;
  }
}

PlannerStatus RRTQuotient::solve(const base::PlannerTerminationCondition &ptc)
{
  Init();

  while (!ptc)
  {
    Grow();

    if(ConnectedToGoal()){
      ConstructSolution(startConfiguration, goalConfiguration);
      isSolved = true;
      break;
    }
  }

  si_->freeState(tgi.xstate);

  OMPL_INFORM("%s: Created %u states (%u start + %u goal)", getName().c_str(), tStart_->size() + tGoal_->size(),
              tStart_->size(), tGoal_->size());

  return (isSolved? base::PlannerStatus::EXACT_SOLUTION : base::PlannerStatus::TIMEOUT);
}

uint RRTQuotient::GetNumberOfVertices(){
  return tStart_->size() + tGoal_->size();
}
uint RRTQuotient::GetNumberOfEdges(){
  return tStart_->size() + tGoal_->size();
}

ompl::PDF<RRTQuotient::Configuration*> RRTQuotient::GetConfigurationPDF()
{
  PDF<Configuration*> pdf;
  std::vector<Configuration *> configurations;
  if(tStart_){
    tStart_->list(configurations);
  }
  for (auto &configuration : configurations)
  {
    if(!(configuration->parent == nullptr)){
      pdf.add(configuration, 1.0/configuration->parent_edge_weight);
    }
  }
  configurations.clear();
  if(tGoal_){
    tGoal_->list(configurations);
  }
  for (auto &configuration : configurations)
  {
    if(!(configuration->parent == nullptr)){
      pdf.add(configuration, 1.0/configuration->parent_edge_weight);
    }
  }

  if(pdf.empty()){
    std::cout << "cannot sample empty(?) graph" << std::endl;
    exit(0);
  }
  return pdf;
}
bool RRTQuotient::SampleGraph(ob::State *q_random_graph)
{

  PDF<Configuration*> pdf = GetConfigurationPDF();

  Configuration *q = pdf.sample(rng_.uniform01());
  double t = rng_.uniform01();
  if(t<0.1){
    double tc = rng_.uniform01();
    const ob::State *from = connectionPoint_.first;
    const ob::State *to = connectionPoint_.second;
    M1->getStateSpace()->interpolate(from, to, tc, q_random_graph);
  }else{
    const ob::State *from = q->state;
    const ob::State *to = q->parent->state;
    M1->getStateSpace()->interpolate(from, to, t, q_random_graph);
  }

  return true;
}
