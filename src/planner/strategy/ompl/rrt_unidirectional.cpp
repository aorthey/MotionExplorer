#include "rrt_unidirectional.h"
#include "elements/plannerdata_vertex_annotated.h"
#include "planner/validitychecker/validity_checker_ompl.h"

using namespace ompl::geometric;

RRTUnidirectional::RRTUnidirectional(const base::SpaceInformationPtr &si, Quotient *previous ): og::Quotient(si, previous)
{
}

RRTUnidirectional::~RRTUnidirectional(void)
{
}

void RRTUnidirectional::Sample(Configuration *q_random){
  if(!hasSolution){
    if(rng_.uniform01() < goalBias_){
      goal->sampleGoal(q_random->state);
    }else{
      sampler_->sampleUniform(q_random->state);
    }
  }else{
    sampler_->sampleUniform(q_random->state);
  }
}

void RRTUnidirectional::setRange(double distance)
{
  maxDistance_ = distance;
}

double RRTUnidirectional::getRange() const
{
  return maxDistance_;
}

uint RRTUnidirectional::GetNumberOfVertices()
{
  return G_->size();
}

uint RRTUnidirectional::GetNumberOfEdges()
{
  std::vector<Configuration *> configs;
  if (G_){
    G_->list(configs);
  }
  uint ctr_edges = 0;
  for (auto &config : configs)
  {
    if (config->parent != nullptr)
    {
      ctr_edges++;
    }
  }
  return ctr_edges;
}


RRTUnidirectional::Configuration* RRTUnidirectional::Nearest(Configuration *q_random)
{
  return G_->nearest(q_random);
}

RRTUnidirectional::Configuration* RRTUnidirectional::Connect(Configuration *q_near, Configuration *q_random)
{
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

    auto checkerPtr = static_pointer_cast<OMPLValidityChecker>(si_->getStateValidityChecker());
    double d1 = checkerPtr->Distance(q_new->state);
    q_new->openset = new cover::OpenSetHypersphere(si_, q_new->state, d1);

    G_->add(q_new);
    return q_new;
  }
  return nullptr;
}

bool RRTUnidirectional::ConnectedToGoal(Configuration* q)
{
  double dist=0.0;
  if(q != nullptr){
    if(goal->isSatisfied(q->state, &dist)){
      hasSolution = true;
      return true;
    }
  }
  return false;
}

void RRTUnidirectional::ConstructSolution(Configuration *q_goal)
{
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

void RRTUnidirectional::Init()
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

  checkValidity();
  goal = dynamic_cast<ob::GoalSampleableRegion *>(pdef_->getGoal().get());

  while (const ob::State *st = pis_.nextStart()){
    auto *q_start = new Configuration(si_);
    si_->copyState(q_start->state, st);

    auto checkerPtr = static_pointer_cast<OMPLValidityChecker>(si_->getStateValidityChecker());
    double d1 = checkerPtr->Distance(q_start->state);
    q_start->openset = new cover::OpenSetHypersphere(si_, q_start->state, d1);

    G_->add(q_start);
  }
  if (G_->size() == 0){
    OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
    exit(0);
  }
  if (!sampler_) sampler_ = si_->allocStateSampler();
}


void RRTUnidirectional::clear()
{
  Planner::clear();
  sampler_.reset();
  freeMemory();
  if(G_){
    G_->clear();
  }
}
void RRTUnidirectional::freeMemory()
{
  if (G_)
  {
    std::vector<Configuration *> configurations;
    G_->list(configurations);
    for (auto &configuration : configurations)
    {
      if (configuration->state != nullptr)
      {
        si_->freeState(configuration->state);
      }
      delete configuration;
    }
  }
}


void RRTUnidirectional::setup(void)
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

void RRTUnidirectional::getPlannerData(base::PlannerData &data) const
{
    //Planner::getPlannerData(data);
    std::vector<Configuration *> vertices;

    if (G_){
      G_->list(vertices);
    }

    if (lastExtendedConfiguration != nullptr){
      data.addGoalVertex(PlannerDataVertexAnnotated(lastExtendedConfiguration->state, 0, lastExtendedConfiguration->openset->GetRadius()));
    }

    for (auto &vertex : vertices)
    {
      double d = vertex->openset->GetRadius();
      if (vertex->parent == nullptr){
        data.addStartVertex(PlannerDataVertexAnnotated(vertex->state, 0, d));
      }else{
        double dp = vertex->parent->openset->GetRadius();
        data.addEdge(PlannerDataVertexAnnotated(vertex->parent->state, 0, dp), PlannerDataVertexAnnotated(vertex->state, 0, d));
      }
      if(!vertex->state){
        std::cout << "vertex state does not exists" << std::endl;
        si_->printState(vertex->state);
      }

      si_->printState(vertex->state);
      std::cout << "vertex with d=" << d << std::endl;
    }
}

void RRTUnidirectional::Grow(double t)
{
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


ob::PlannerStatus RRTUnidirectional::solve(const ob::PlannerTerminationCondition &ptc)
{
  Init();

  while(!ptc){

    Grow();

    if(HasSolution())
    {
      ConstructSolution(lastExtendedConfiguration);
      return ob::PlannerStatus::EXACT_SOLUTION;
    }
  }

  OMPL_INFORM("%s: Created %u states", getName().c_str(), G_->size());
  return ob::PlannerStatus::TIMEOUT;
}

bool RRTUnidirectional::HasSolution()
{
  if(!hasSolution){
    if(ConnectedToGoal(lastExtendedConfiguration)){
      hasSolution = true;
    }
  }
  return hasSolution;
}

void RRTUnidirectional::CheckForSolution(ob::PathPtr &solution)
{

  if(!hasSolution) return;

  Configuration *q_goal = lastExtendedConfiguration;
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
    solution = path;
    goalBias_ = 0.0;
  }
}
