#include "common.h"
#include "qst.h"
#include "elements/plannerdata_vertex_annotated.h"
#include "planner/cspace/validitychecker/validity_checker_ompl.h"

using namespace ompl::geometric;

QST::QST(const base::SpaceInformationPtr &si, Quotient *parent ): BaseT(si, parent)
{
  setName("QST"+std::to_string(id));
}

QST::~QST(void)
{
}

void QST::AddConfiguration(Configuration *q)
{
  bool feasible = Q1->isValid(q->state);

  if(feasible){

    if(IsOuterRobotFeasible(q->state))
    {
      q->isSufficientFeasible = true;
      q->openNeighborhoodRadius = DistanceOuterRobotToObstacle(q->state);
    }else{
      q->openNeighborhoodRadius = DistanceInnerRobotToObstacle(q->state);
      pdf_necessary_vertices.add(q, q->openNeighborhoodRadius);
    }
    pdf_all_vertices.add(q, q->openNeighborhoodRadius);

    //why do we need the nearestneighbors structure here?
    cspace_tree->add(q);
  }

}
void QST::AddState(ob::State *state)
{
  Configuration *q = new Configuration(Q1, state);
  AddConfiguration(q);
}


void QST::Grow(double t)
{
  Configuration *q_random = new Configuration(si_);

  Sample(q_random);

  bool feasible = Q1->isValid(q_random->state);

  if(feasible){

    if(IsOuterRobotFeasible(q_random->state))
    {
      q_random->isSufficientFeasible = true;
      q_random->openNeighborhoodRadius = DistanceOuterRobotToObstacle(q_random->state);
    }else{
      q_random->openNeighborhoodRadius = DistanceInnerRobotToObstacle(q_random->state);
      pdf_necessary_vertices.add(q_random, q_random->openNeighborhoodRadius);
    }
    pdf_all_vertices.add(q_random, q_random->openNeighborhoodRadius);

    //why do we need the nearestneighbors structure here?
    cspace_tree->add(q_random);
  }

  if(q_random->state != nullptr){
    si_->freeState(q_random->state);
  }
  delete q_random;
}


double QST::Distance(const Configuration *q_from, const Configuration *q_to)
{
  double d = si_->distance(q_from->state, q_to->state);
  return d;
}

void QST::Sample(Configuration *q_random){
  if(parent == nullptr){
    //Q1_sampler->sampleUniform(q_random->state);
    Configuration *q = pdf_all_vertices.sample(rng_.uniform01());
    double radius = q->openNeighborhoodRadius;
    uint dimension = Q1->getStateDimension();

    ob::State *qk = Q1->allocState();
    Q1_sampler->sampleGaussian(qk, q->state, 1);

    std::cout << radius << std::endl;
    std::cout << dimension << std::endl;
    std::cout << "Need to sample from ball of radius " << radius << ": NIY" << std::endl;
    exit(0);

    //how to sample on the boundary!?


  }else{
    std::cout << "NIY" << std::endl;
    exit(0);
    // ob::SpaceInformationPtr Q0 = parent->getSpaceInformation();
    // base::State *s_X1 = X1->allocState();
    // base::State *s_Q0 = Q0->allocState();

    // X1_sampler->sampleUniform(s_X1);
    // parent->SampleGraph(s_Q0);
    // mergeStates(s_Q0, s_X1, q_random->state);

    // X1->freeState(s_X1);
    // Q0->freeState(s_Q0);
  }
}

uint QST::GetNumberOfVertices() const
{
  return cspace_tree->size();
}

uint QST::GetNumberOfEdges() const
{
  std::vector<Configuration *> configs;
  if (cspace_tree){
    cspace_tree->list(configs);
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

// QST::Configuration* QST::Nearest(Configuration *q)
// {
//   return cspace_tree->nearest(q);
// }

// QST::Configuration* QST::Connect(Configuration *q_from, Configuration *q_to)
// {
//   //two strategies: either check directly if the edge between q_near and
//   //q_random is feasible and add that (as implemented by RRTConnect)
//   //or: move as long towards q_random as the edge is feasible. Once it becomes
//   //infeasible, set q_random to the last feasible vertex. Might be more
//   //efficient, but why is that not implemented in RRTConnect?
//   //=> it seems we often need to reject samples, especially if the boundary box
//   //of the workspace is not tight. 

//   double d = Distance(q_from, q_to);

//   //##############################################################################
//   // move towards q_to until infeasible
//   //##############################################################################
//   double d_segment = si_->getStateSpace()->getLongestValidSegmentFraction();
//   //uint nd = si_->getStateSpace()->validSegmentCount(q_near->state, q_random->state);

//   double d_cum = 0;
//   double d_lastvalid = 0;
//   ob::State *q_test = si_->allocState();
//   while(d_cum <= d)
//   {
//     si_->getStateSpace()->interpolate(q_from->state, q_to->state, d_cum / d, q_test);
//     if(si_->isValid(q_test)){
//       d_lastvalid = d_cum;
//     }else{
//       break;
//     }
//     d_cum += d_segment;
//   }
//   si_->freeState(q_test);

//   if(d_lastvalid<=0) return nullptr;

//   auto *q_new = new Configuration(si_);
//   q_new->state = si_->allocState();

//   si_->getStateSpace()->interpolate(q_from->state, q_to->state, d_lastvalid / d, q_new->state);

//   //si_->copyState(q_new->state, q_to->state);
//   q_new->parent = q_from;
//   q_new->parent_edge_weight = Distance(q_from->state, q_new->state);
//   graphLength += q_new->parent_edge_weight;

//   cspace_tree->add(q_new);

//   return q_new;
// }

bool QST::ConnectedToGoal(Configuration* q)
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

void QST::ConstructSolution(Configuration *configuration_goal)
{
  if (configuration_goal != nullptr){

    std::vector<Configuration *> q_path;
    while (configuration_goal != nullptr){
      q_path.push_back(configuration_goal);
      configuration_goal = configuration_goal->parent;
    }

    auto path(std::make_shared<PathGeometric>(si_));
    for (int i = q_path.size() - 1; i >= 0; --i){
      path->append(q_path[i]->state);
    }
    pdef_->addSolutionPath(path);
  }
}

void QST::Init()
{

  checkValidity();
}


void QST::clear()
{
  Planner::clear();
  freeMemory();
  if(cspace_tree){
    cspace_tree->clear();
  }
  hasSolution = false;
  lastExtendedConfiguration = nullptr;
  q_start = nullptr;
  q_goal = nullptr;

  pis_.restart();
}

void QST::freeMemory()
{
  if (cspace_tree)
  {
    std::vector<Configuration *> configurations;
    cspace_tree->list(configurations);
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

void QST::setup(void)
{
  if (!cspace_tree){
    cspace_tree.reset(tools::SelfConfig::getDefaultNearestNeighbors<Configuration *>(this));
    cspace_tree->setDistanceFunction([this](const Configuration *a, const Configuration *b)
                             {
                                return Distance(a,b);
                                });
  }

  if (pdef_){
    if (pdef_->hasOptimizationObjective()){
      opt_ = pdef_->getOptimizationObjective();
    }else{
      OMPL_ERROR("%s: Did not specify optimization function.", getName().c_str());
      exit(0);
    }
    if(const ob::State *state = pis_.nextStart()){
      q_start = new Configuration(si_, state);
      AddConfiguration(q_start);
    }else{
      OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
      exit(0);
    }
    if(const ob::State *state = pis_.nextGoal()){
      q_goal = new Configuration(si_, state);
    }else{
      OMPL_ERROR("%s: There are no valid goal states!", getName().c_str());
      exit(0);
    }
    
    OMPL_INFORM("%s: ready with %lu states already in datastructure", getName().c_str(), cspace_tree->size());
    setup_ = true;
  }else{
    setup_ = false;
  }

}
void QST::getPlannerData(base::PlannerData &data) const
{
  std::vector<Configuration *> vertices;

  if (cspace_tree){
    cspace_tree->list(vertices);
  }

  if (lastExtendedConfiguration != nullptr){
    data.addGoalVertex(PlannerDataVertexAnnotated(lastExtendedConfiguration->state, 0, lastExtendedConfiguration->GetRadius()));
  }

  for (auto &vertex : vertices)
  {
    double d = vertex->GetRadius();
    if (vertex->parent == nullptr){
      data.addStartVertex(PlannerDataVertexAnnotated(vertex->state, 0, d));
    }else{
      double dp = vertex->parent->GetRadius();
      data.addEdge(PlannerDataVertexAnnotated(vertex->parent->state, 0, dp), PlannerDataVertexAnnotated(vertex->state, 0, d));
    }
    if(!vertex->state){
      std::cout << "vertex state does not exists" << std::endl;
      si_->printState(vertex->state);
      exit(0);
    }
  }
}

bool QST::HasSolution()
{
  if(!hasSolution){
    if(ConnectedToGoal(lastExtendedConfiguration)){
      hasSolution = true;
    }
  }
  return hasSolution;
}

void QST::CheckForSolution(ob::PathPtr &solution)
{
  if(!hasSolution) return;

  Configuration *configuration_goal = lastExtendedConfiguration;
  if (configuration_goal != nullptr){

    std::vector<Configuration *> q_path;
    while (configuration_goal != nullptr){
      q_path.push_back(configuration_goal);
      configuration_goal = configuration_goal->parent;
    }

    auto path(std::make_shared<PathGeometric>(si_));
    for (int i = q_path.size() - 1; i >= 0; --i){
      path->append(q_path[i]->state);
    }
    solution = path;
  }
}
bool QST::SampleGraph(ob::State *q_random_graph)
{
  std::cout << "NIY" << std::endl;
  exit(0);
  return false;
  //Configuration *q = pdf.sample(rng_.uniform01());
  //double t = rng_.uniform01();

  //const ob::State *q_from = q->state;
  //const ob::State *q_to = q->parent->state;
  //Q1->getStateSpace()->interpolate(q_from, q_to, t, q_random_graph);

  //lastSampled = q;
  //lastSampled->totalSamples++;
  //return true;
}
