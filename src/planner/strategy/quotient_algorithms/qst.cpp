#include "common.h"
#include "qst.h"
#include "elements/plannerdata_vertex_annotated.h"
#include "planner/cspace/validitychecker/validity_checker_ompl.h"

using namespace ompl::geometric;

QST::QST(const base::SpaceInformationPtr &si, Quotient *parent ): BaseT(si, parent)
{
  setName("QST"+std::to_string(id));
  if (!cover_tree){
    cover_tree.reset(tools::SelfConfig::getDefaultNearestNeighbors<Configuration *>(this));
    cover_tree->setDistanceFunction([this](const Configuration *a, const Configuration *b)
                             {
                                return OpenNeighborhoodDistance(a,b);
                              });
  }
}

QST::~QST(void)
{
}

void QST::setup(void)
{

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
    
    OMPL_INFORM("%s: ready with %lu states already in datastructure", getName().c_str(), cover_tree->size());
    setup_ = true;
  }else{
    setup_ = false;
  }

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
    cover_tree->add(q);
  }else{
    if(q->state != nullptr){
      si_->freeState(q->state);
    }
    delete q;
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

  Configuration *q_nearest = Nearest(q_random);

  Connect(q_nearest, q_random);

  AddConfiguration(q_random);

  if(q_random != nullptr)
  {
    q_random->parent = q_nearest;
  }

}

bool QST::Sample(Configuration *q_random){
  if(parent == nullptr){
    if(!hasSolution && rng_.uniform01() < goalBias){
      q_random->state = si_->cloneState(q_goal->state);
    }else{
      Q1_sampler->sampleUniform(q_random->state);
    }
      // Configuration *q = pdf_all_vertices.sample(rng_.uniform01());
      // sampleUniformOnNeighborhoodBoundary(q_random, q);

      //check that the sampled configuration is not inside the cover, but on
      //boundary
      // std::vector<Configuration*> neighbors;
      // cover_tree->nearestR(q_random, 1e-10, neighbors);
    return true;
  }else{
    std::cout << "NIY" << std::endl;
    exit(0);
    return false;
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

QST::Configuration* QST::Nearest(Configuration *q) const
{
  return cover_tree->nearest(q);
}

void QST::Connect(const Configuration *q_from, Configuration *q_to)
{
  double d = Distance(q_from, q_to);
  double radius = q_from->GetRadius();
  si_->getStateSpace()->interpolate(q_from->state, q_to->state, radius / d, q_to->state);
}

double QST::Distance(const Configuration *q_from, const Configuration *q_to)
{
  double d = si_->distance(q_from->state, q_to->state);
  return d;
}

double QST::OpenNeighborhoodDistance(const Configuration *q_from, const Configuration *q_to)
{
  double d_from = q_from->GetRadius();
  double d_to = q_to->GetRadius();
  double d = si_->distance(q_from->state, q_to->state);

  //note that this is a pseudometric: invalidates second axiom of metric : d(x,y) = 0  iff x=y. But here we only have d(x,x)=0
  double d_open_neighborhood_distance = std::max(d - d_from - d_to, 0.0); 
  return d_open_neighborhood_distance;
}

bool QST::sampleUniformOnNeighborhoodBoundary(Configuration *sample, const Configuration *center)
{
  //sample on boundary of open neighborhood
  // (1) first sample gaussian point qk around q
  // (2) project qk onto neighborhood
  // (*) this works because gaussian is symmetric around origin
  // (*) this does not work when qk is near to q, so we need to sample as long
  // as it is not near
  //
  double radius = center->openNeighborhoodRadius;

  double dist_q_qk = 0;
  double epsilon_min_distance = 1e-10;

  //@DEBUG
  if(epsilon_min_distance >= radius){
    OMPL_ERROR("neighborhood is too small to sample the boundary.");
    std::cout << "neighborhood radius: " << radius << std::endl;
    std::cout << "minimal distance to be able to sample: " << epsilon_min_distance << std::endl;
    exit(0);
  }

  //sample as long as we are not outside the ball of radius epsilon_min_distance
  while(dist_q_qk < epsilon_min_distance){
    Q1_sampler->sampleGaussian(sample->state, center->state, 1);
    dist_q_qk = si_->distance(center->state, sample->state);
  }
  si_->getStateSpace()->interpolate(center->state, sample->state, radius/dist_q_qk, sample->state);

  return true;
}


uint QST::GetNumberOfVertices() const
{
  return cover_tree->size();
}

uint QST::GetNumberOfEdges() const
{
  std::vector<Configuration *> configs;
  if (cover_tree){
    cover_tree->list(configs);
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

void QST::Init()
{

  checkValidity();
}


void QST::clear()
{
  Planner::clear();
  freeMemory();
  if(cover_tree){
    cover_tree->clear();
  }
  hasSolution = false;
  q_start = nullptr;
  q_goal = nullptr;

  pis_.restart();
}

void QST::freeMemory()
{
  if (cover_tree)
  {
    std::vector<Configuration *> configurations;
    cover_tree->list(configurations);
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

void QST::CheckForSolution(ob::PathPtr &solution)
{
  Configuration *q_nearest = Nearest(q_goal);
  if(OpenNeighborhoodDistance(q_nearest, q_goal) <= 0)
  {
    if(q_goal->parent == nullptr){
      q_goal->parent = q_nearest;
      cover_tree->add(q_goal);
    }

    Configuration *q_next = q_goal;
    std::vector<Configuration *> q_path;
    while (q_next != nullptr){
      q_path.push_back(q_next);
      q_next = q_next->parent;
    }
    auto path(std::make_shared<PathGeometric>(si_));
    for (int i = q_path.size() - 1; i >= 0; --i){
      path->append(q_path.at(i)->state);
    }
    solution = path;

    hasSolution = true;
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

void QST::SetSubGraph( QuotientChart *sibling, uint k )
{
  local_chart = true;
  opt_ = sibling->opt_;
  level = sibling->GetLevel();
  startM_ = sibling->startM_;
  goalM_ = sibling->goalM_;
  number_of_paths = 0;

  std::cout << "NYI" << std::endl;
  exit(0);
  std::cout << "new graph: " << GetNumberOfVertices() << " vertices | " << GetNumberOfEdges() << " edges."<< std::endl;
}
void QST::getPlannerData(base::PlannerData &data) const
{
  uint Nvertices = data.numVertices();
  uint Nedges = data.numEdges();
  //###########################################################################
  //Get Path for this chart
  //###########################################################################
  std::vector<int> path = GetPath();

  //###########################################################################
  //Obtain vertices
  //###########################################################################
  std::vector<Configuration *> vertices;
  if (cover_tree){
    cover_tree->list(vertices);
  }

  //if (lastExtendedConfiguration != nullptr){
  //data.addGoalVertex(pvertex);

  for (auto &vertex : vertices)
  {
    ob::State *state = (local_chart?si_->cloneState(vertex->state):vertex->state);
    double d = vertex->GetRadius();

    PlannerDataVertexAnnotated pvertex(state);
    pvertex.SetLevel(level);
    pvertex.SetPath(path);
    pvertex.SetOpenNeighborhoodDistance(d);

    using FeasibilityType = PlannerDataVertexAnnotated::FeasibilityType;
    if(vertex->isSufficientFeasible){
      pvertex.SetFeasibility(FeasibilityType::SUFFICIENT_FEASIBLE);
    }else{
      pvertex.SetFeasibility(FeasibilityType::FEASIBLE);
    }

    if(vertex->parent == nullptr){
      data.addStartVertex(pvertex);
    }else{
      if(vertex == q_goal){
        data.addGoalVertex(pvertex);
      }else{
        data.addVertex(pvertex);
      }

      PlannerDataVertexAnnotated parent_vertex(vertex->parent->state);
      parent_vertex.SetLevel(level);
      parent_vertex.SetPath(path);
      parent_vertex.SetOpenNeighborhoodDistance(vertex->parent->GetRadius());
      data.addEdge(pvertex, parent_vertex);
    }
    if(!vertex->state){
      std::cout << "vertex state does not exists" << std::endl;
      si_->printState(vertex->state);
      exit(0);
    }
  }
  //###########################################################################
  //Get Data From all siblings
  //###########################################################################

  std::cout << "[QuotientChart] vIdx " << level << " | hIdx " << horizontal_index 
    << " | siblings " << siblings.size() << " | path " << path 
    << " | vertices " << data.numVertices() - Nvertices 
    << " | edges " << data.numEdges() - Nedges
    << std::endl;

  for(uint i = 0; i < siblings.size(); i++){
    siblings.at(i)->getPlannerData(data);
  }
  if(child!=nullptr) child->getPlannerData(data);
}

