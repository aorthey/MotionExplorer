#include "rrt_bidirectional.h"
#include "planner/validitychecker/validity_checker_ompl.h"
#include "elements/plannerdata_vertex_annotated.h"
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/tools/config/SelfConfig.h>
#include <ompl/base/PlannerData.h>

using namespace ompl::geometric;
using namespace ompl::base;
using namespace og;

RRTBidirectional::RRTBidirectional(const base::SpaceInformationPtr &si, Quotient *previous_) : 
  Quotient(si, previous_)
{
  setName("RRTBidirectional");
  specs_.recognizedGoal = base::GOAL_SAMPLEABLE_REGION;
  specs_.directed = true;
  Planner::declareParam<double>("range", this, &RRTBidirectional::setRange, &RRTBidirectional::getRange, "0.:1.:10000.");
  connectionPoint_ = std::make_pair<base::State *, base::State *>(nullptr, nullptr);
  startTree = true;
}

RRTBidirectional::~RRTBidirectional()
{
  freeMemory();
}

void RRTBidirectional::setRange(double distance)
{
  maxDistance_ = distance;
}

double RRTBidirectional::getRange() const
{
  return maxDistance_;
}

void RRTBidirectional::setup()
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
  tgi.xstate = si_->allocState();
}

void RRTBidirectional::getPlannerData(base::PlannerData &data) const
{
    std::vector<Configuration *> configurations;
    if (tStart_)
        tStart_->list(configurations);

    uint startComponent = 0;
    uint goalComponent = 1;
    if(isTreeConnected) goalComponent = startComponent; 

    for (auto &configuration : configurations)
    {
        if (configuration->parent == nullptr)
        {
          PlannerDataVertexAnnotated v(configuration->state, 1, configuration->openNeighborhoodDistance);
          v.SetComponent(startComponent);
          data.addStartVertex(v);
        }else
        {
          double d = configuration->openNeighborhoodDistance;
          double dp = configuration->parent->openNeighborhoodDistance;
          PlannerDataVertexAnnotated v1(configuration->state, 1, d);
          PlannerDataVertexAnnotated v2(configuration->parent->state, 1, dp);
          v1.SetComponent(startComponent);
          v2.SetComponent(startComponent);
          data.addEdge(v2, v1);
        }
    }

    configurations.clear();
    if (tGoal_)
        tGoal_->list(configurations);

    for (auto &configuration : configurations)
    {
        if (configuration->parent == nullptr)
        {
          PlannerDataVertexAnnotated v(configuration->state, 2, configuration->openNeighborhoodDistance);
          v.SetComponent(goalComponent);
          data.addGoalVertex(v);
        }else
        {
          double d = configuration->openNeighborhoodDistance;
          double dp = configuration->parent->openNeighborhoodDistance;
          PlannerDataVertexAnnotated v1(configuration->state, 2, d);
          PlannerDataVertexAnnotated v2(configuration->parent->state, 2, dp);
          v1.SetComponent(goalComponent);
          v2.SetComponent(goalComponent);
          data.addEdge(v1, v2);
        }
    }

    data.addEdge(data.vertexIndex(connectionPoint_.first), data.vertexIndex(connectionPoint_.second));

    //std::cout << std::string(80, '-') << std::endl;
    //for(uint k = 0; k < data.numVertices(); k++){
    //  PlannerDataVertexAnnotated v = *static_cast<PlannerDataVertexAnnotated*>(&data.getVertex(k));
    //  std::cout << v.GetOpenNeighborhoodDistance() << std::endl;
    //}
    //std::cout << std::string(80, '-') << std::endl;
}


void RRTBidirectional::freeMemory()
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

void RRTBidirectional::clear()
{
  Quotient::clear();
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

RRTBidirectional::GrowState RRTBidirectional::growTree(TreeData &tree, TreeGrowingInfo &tgi,
                                                                             Configuration *rconfiguration)
{
  Configuration *nconfiguration = tree->nearest(rconfiguration);

  bool reach = true;

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

    auto checkerPtr = static_pointer_cast<OMPLValidityChecker>(si_->getStateValidityChecker());
    configuration->openNeighborhoodDistance = checkerPtr->Distance(configuration->state);

    if(checkerPtr->IsSufficient(configuration->state))
    {
      configuration->isSufficient = true;
      configuration->parent_edge_weight = 0;
    }

    tgi.xconfiguration = configuration;
    tree->add(configuration);
    return reach ? REACHED : ADVANCED;
  }
  else
    return TRAPPED;
}
void RRTBidirectional::Init(){
  checkValidity();
  goal = dynamic_cast<ob::GoalSampleableRegion *>(pdef_->getGoal().get());
  if (goal == nullptr)
  {
      OMPL_ERROR("%s: Unknown type of goal", getName().c_str());
      exit(0);
  }
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

}

ob::PathPtr RRTBidirectional::ConstructSolution(Configuration *q_start, Configuration *q_goal){
  //this removes a double point, but it will create some problems with the PDF
  //further down, so we leave it
  if (q_start->parent != nullptr){
    q_start = q_start->parent;
  }else{
    q_goal = q_goal->parent;
  }

  connectionPoint_ = std::make_pair(q_start->state, q_goal->state);

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



void RRTBidirectional::Grow(double t)
{
  TreeData &tree = startTree ? tStart_ : tGoal_;
  tgi.start = startTree;
  startTree = !startTree;
  TreeData &otherTree = startTree ? tStart_ : tGoal_;

  Configuration *q_random = new Configuration(M1);
  //q_random->state = M1->allocState();
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
  delete q_random;
}
bool RRTBidirectional::ConnectedToGoal(){
  return isTreeConnected && goal->isStartGoalPairValid(startConfiguration->root, goalConfiguration->root);
}

void RRTBidirectional::CheckForSolution(ob::PathPtr &solution)
{
  if(!isSolved && ConnectedToGoal()){
    solution = ConstructSolution(startConfiguration, goalConfiguration);
    isSolved = true;
  }
}

PlannerStatus RRTBidirectional::solve(const base::PlannerTerminationCondition &ptc)
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

uint RRTBidirectional::GetNumberOfVertices(){
  return tStart_->size() + tGoal_->size();
}
uint RRTBidirectional::GetNumberOfEdges(){
  return tStart_->size() + tGoal_->size();
}

ompl::PDF<RRTBidirectional::Configuration*> RRTBidirectional::GetConfigurationPDF()
{
  PDF<Configuration*> pdf;
  double t = rng_.uniform01();
  if(t<0.5){
    //shortest path heuristic
    Configuration *configuration = startConfiguration;
    while (configuration->parent != nullptr)
    {
      pdf.add(configuration, configuration->parent_edge_weight);
      configuration = configuration->parent;
    }

    configuration = goalConfiguration;
    while (configuration->parent != nullptr)
    {
      pdf.add(configuration, configuration->parent_edge_weight);
      configuration = configuration->parent;
    }

  }else{
    std::vector<Configuration *> configurations;
    if(tStart_){
      tStart_->list(configurations);
    }
    for (auto &configuration : configurations)
    {
      if(!(configuration->parent == nullptr)){
        pdf.add(configuration, configuration->parent_edge_weight);
      }
    }
    configurations.clear();
    if(tGoal_){
      tGoal_->list(configurations);
    }
    for (auto &configuration : configurations)
    {
      if(!(configuration->parent == nullptr)){
        pdf.add(configuration, configuration->parent_edge_weight);
      }
    }
  }

  if(pdf.empty()){
    std::cout << "cannot sample empty(?) graph" << std::endl;
    exit(0);
  }
  return pdf;
}
bool RRTBidirectional::SampleGraph(ob::State *q_random_graph)
{

  PDF<Configuration*> pdf = GetConfigurationPDF();
  Configuration *q = pdf.sample(rng_.uniform01());
  double t = rng_.uniform01();

  const ob::State *from = q->state;
  const ob::State *to = q->parent->state;
  M1->getStateSpace()->interpolate(from, to, t, q_random_graph);
  double epsilon = 0.05;
  M1_sampler->sampleUniformNear(q_random_graph, q_random_graph, epsilon);

  //auto checkerPtr = static_pointer_cast<OMPLValidityCheckerNecessarySufficient>(M1->getStateValidityChecker());
  //bool foundNecessary = false;
  //while(!foundNecessary)
  //{
  //  Configuration *q = pdf.sample(rng_.uniform01());
  //  double t = rng_.uniform01();

  //  const ob::State *from = q->state;
  //  const ob::State *to = q->parent->state;
  //  M1->getStateSpace()->interpolate(from, to, t, q_random_graph);

  //  if(!checkerPtr->IsSufficient(q_random_graph)){
  //    foundNecessary = true;
  //    double epsilon = 5;
  //    simpleSampler_->sampleGaussian(q_random_graph, q_random_graph, epsilon);
  //  }
  //}

  return true;
}
