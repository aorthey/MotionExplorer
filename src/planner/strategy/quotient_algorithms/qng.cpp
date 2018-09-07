#include "common.h"
#include "qng.h"
#include "elements/plannerdata_vertex_annotated.h"
#include "planner/cspace/validitychecker/validity_checker_ompl.h"
#include <limits>


using namespace ompl::geometric;

QNG::QNG(const base::SpaceInformationPtr &si, Quotient *parent ): BaseT(si, parent)
{
  setName("QNG"+std::to_string(id));
  if (!nearest_cover){
    nearest_cover.reset(tools::SelfConfig::getDefaultNearestNeighbors<Configuration *>(this));
    nearest_cover->setDistanceFunction([this](const Configuration *a, const Configuration *b)
                             {
                                return DistanceOpenNeighborhood(a,b);
                              });
  }
  if (!nearest_vertex){
    nearest_vertex.reset(tools::SelfConfig::getDefaultNearestNeighbors<Configuration *>(this));
    nearest_vertex->setDistanceFunction([this](const Configuration *a, const Configuration *b)
                             {
                                return DistanceQ1(a,b);
                              });
  }
}

QNG::~QNG(void)
{
}

void QNG::setup(void)
{

  if (pdef_){
    //#########################################################################
    if (pdef_->hasOptimizationObjective()){
      opt_ = pdef_->getOptimizationObjective();
    }else{
      OMPL_ERROR("%s: Did not specify optimization function.", getName().c_str());
      exit(0);
    }
    //#########################################################################
    //Adding start configuration
    //#########################################################################
    if(const ob::State *state = pis_.nextStart()){
      Configuration *coset = nullptr;
      if(parent != nullptr)
      {
        coset = static_cast<og::QNG*>(parent)->q_start;
      }
      q_start = AddState(state, coset);

      if(q_start == nullptr){
        OMPL_ERROR("%s: Could not add start state!", getName().c_str());
        exit(0);
      }
    }else{
      OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
      exit(0);
    }

    //#########################################################################
    //Adding goal configuration
    //#########################################################################
    if(const ob::State *state = pis_.nextGoal()){
      Configuration *coset = nullptr;
      if(parent != nullptr)
      {
        coset = static_cast<og::QNG*>(parent)->q_goal;
      }
      q_goal = AddState(state, coset);
      if(q_goal == nullptr){
        OMPL_ERROR("%s: Could not add goal state!", getName().c_str());
        exit(0);
      }
    }else{
      OMPL_ERROR("%s: There are no valid goal states!", getName().c_str());
      exit(0);
    }

    //double distance_start_to_goal = DistanceQ1(q_start, q_goal);
    if(q_start->GetRadius() == std::numeric_limits<double>::infinity())
    {
      OMPL_INFORM("Note: start state covers quotient-space.");
      saturated = true;
    }
    //#########################################################################
    OMPL_INFORM("%s: ready with %lu states already in datastructure", getName().c_str(), nearest_cover->size());
    setup_ = true;
  }else{
    setup_ = false;
  }

}

void QNG::AddConfiguration(Configuration *q)
{
  Vertex v = boost::add_vertex(*q, graph);
  G[v].total_connection_attempts = 1;
  G[v].successful_connection_attempts = 0;
  disjointSets_.make_set(v);

  PDF_Element *q_element = pdf_all_configurations.add(q, q->GetImportance());
  q->SetPDFElement(q_element);
  nearest_cover->add(q);
  nearest_vertex->add(q);
}

QNG::Configuration* QNG::AddState(const ob::State *state, Configuration *q_coset)
{

  Configuration *q = new Configuration(Q1, state);
  if(IsSampleInsideCover(q)){
    q->Remove(Q1);
    return nullptr;
  }

  q->coset = q_coset;

  bool feasible = Q1->isValid(q->state);

  if(feasible){
    if(IsOuterRobotFeasible(q->state))
    {
      q->isSufficientFeasible = true;
      q->SetRadius(DistanceOuterRobotToObstacle(q->state));
    }else{
      q->SetRadius(DistanceInnerRobotToObstacle(q->state));
    }

    if(q->GetRadius()<threshold_clearance){
      q->Remove(Q1);
      return nullptr;
    }

    //sample lies outside our cover, but some samples might now be contained by
    //q
    RemoveCoveredSamples(q);

    //add to structures
    if(!q->isSufficientFeasible){
      pdf_necessary_configurations.add(q, q->GetRadius());
    }
  }else{
    q->Remove(Q1);
    return nullptr;
  }

  AddConfiguration(q);
  return q;
}


//if new sample is inside the current cover, return true
//we check the two nearest points. only one of them can be a parent, ignore that
//one, and check distance of the second one.

bool QNG::IsSampleInsideCover(Configuration *q)
{
  std::vector<Configuration*> neighbors;

  nearest_cover->nearestK(q, 2, neighbors);

  for(uint k = 0; k < neighbors.size(); k++)
  {
    //Configuration *qn = neighbors.at(k);
    //the configuration is allowed to be on the boundary of the parent
    std::cout << "NYI" << std::endl;
    exit(0);
    // if(qn != q->parent){

    //   double dqn = DistanceQ1(qn, q);
    //   double radius_n = qn->GetRadius();
    //   if( dqn < radius_n ){
    //     //std::cout << "distance rejected: " << dqn << " < " << radius_n << std::endl;
    //     return true;
    //   }
    // }
  }
  return false;
}

//If the neighborhood of q is a superset of any other neighborhood, then delete
//the other neighborhood, and rewire the tree.
void QNG::RemoveCoveredSamples(Configuration *q)
{
  double radius_q = q->GetRadius();
  std::vector<Configuration*> neighbors;

  //get all vertices inside open set of q
  nearest_vertex->nearestR(q, radius_q, neighbors);

  std::cout << "NYI" << std::endl;
  exit(0);
  // for(uint k = 0; k < neighbors.size(); k++){

  //   Configuration *qn = neighbors.at(k);

  //   if(qn->parent == nullptr) continue; //start configuration
  //   //if(qn == q->parent) continue; //parent configuration

  //   double distance_q_qn = DistanceQ1(q, qn);
  //   double radius_k = qn->GetRadius();

  //   //if(radius_q > distance_q_qn){
  //   if(radius_q > radius_k+distance_q_qn){
  //     RemoveConfiguration(qn);
  //   }
  // }
}

void QNG::Grow(double t)
{
  return;
  //#########################################################################
  //Do not grow the cover if it is saturated, i.e. it cannot be expanded anymore
  // --- we have successfully computed Q1_{free}, the free space of Q1
  //#########################################################################
  if(saturated) return;

  //#########################################################################
  //Sample a configuration different from the current cover
  //#########################################################################

  // Configuration *q_random = new Configuration(si_);
  // Sample(q_random);

  // //#########################################################################
  // //Nearest in cover
  // //#########################################################################
  // Configuration *q_nearest = Nearest(q_random);
  // if(q_nearest->state == q_random->state){
  //   OMPL_ERROR("sampled and nearest states are equivalent.");
  //   std::cout << "sampled state" << std::endl;
  //   Q1->printState(q_random->state);
  //   std::cout << "nearest state" << std::endl;
  //   Q1->printState(q_nearest->state);
  //   exit(0);
  // }

  // //#########################################################################
  // //Connect nearest to random --- while staying in neighborhood of q_nearest
  // //#########################################################################
  // Connect(q_nearest, q_random);

  // //#########################################################################
  // //Try to expand further in that direction until we hit an infeasible point, or
  // //the time is up
  // //
  // //  q_nearest  ---->  q_random  ---->  q_next
  // //#########################################################################
  // if(AddConfiguration(q_random, q_nearest)){
  //   const uint max_extension_steps = 1000;
  //   uint step = 0;
  //   while(step++ < max_extension_steps)
  //   {
  //     q_random->number_attempted_expansions++;
  //     pdf_all_configurations.update(static_cast<PDF_Element*>(q_random->GetPDFElement()), q_random->GetImportance());

  //     q_nearest = q_random->parent;
  //     Configuration *q_next = EstimateBestNextState(q_nearest, q_random);

  //     if(q_next == nullptr)
  //     {
  //       if(verbose>1) std::cout << "q_next is nullptr" << std::endl;
  //       break;
  //     }
  //     //note that q_random might have been removed if the neighborhood is a
  //     //proper subset of the neighborhood of q_next
  //     if(q_random == nullptr){
  //       Connect(q_nearest, q_next);
  //     }else{
  //       Connect(q_random, q_next);
  //       pdf_all_configurations.update(static_cast<PDF_Element*>(q_random->GetPDFElement()), q_random->GetImportance());
  //     }
  //     q_random = q_next;
  //   }
  // }

  // //update PDF
  // if(q_nearest != nullptr){
  //   pdf_all_configurations.update(static_cast<PDF_Element*>(q_nearest->GetPDFElement()), q_nearest->GetImportance());
  // }
}
QNG::Configuration* QNG::EstimateBestNextState(Configuration *q_last, Configuration *q_current)
{
    uint K_samples = 3; //how many samples to test for best direction (depends maybe also on radius)

    Configuration *q_next = new Configuration(Q1);

    if(verbose>1){
      std::cout << "from" << std::endl;
      Q1->printState(q_last->state);
      std::cout << "to" << std::endl;
      Q1->printState(q_current->state);
    }

    double d_last_to_current = Distance(q_last, q_current);
    double radius_current = q_current->GetRadius();
    Q1->getStateSpace()->interpolate(q_last->state, q_current->state, 1 + radius_current/d_last_to_current, q_next->state);

    //#######################################################################
    // We sample the distance function to obtain K samples {q_1,\cdots,q_K}.
    // For each $q_k$ we compute the value of the distance function dk_radius.
    // Then we employ two strategies to choose the next sample to follow:
    //
    //either follow isolines : min( fabs(radius_k_sample - current_radius) )
    //or follow the steepest ascent: max(radius_k_sample)

    double radius_best = DistanceInnerRobotToObstacle(q_next->state);
    if(verbose>1){
      std::cout << "next" << std::endl;
      Q1->printState(q_next->state);
      std::cout << "radius " << radius_best << std::endl;
    }

    //double radius_best = q_next->GetRadius();
    double SAMPLING_RADIUS = 0.1*radius_current;
    double radius_ratio = radius_best / radius_current;
    if(radius_ratio > 1){
      //we encountered a bigger radius neighborhood. this is great, we should go
      //into that direction
      if(!AddConfiguration(q_next, q_current, true))
      {
        return nullptr;
      }
      return q_next;
    }else{
      //if next neighborhood is exceptionally small, try to sample more broadly.
      //Otherwise sample smaller. This corresponds to having more speed in the
      //momentum of sampling.
      if(radius_ratio > 0.1){
        SAMPLING_RADIUS = (1.0-radius_ratio+0.1)*radius_current;
      }else{
        SAMPLING_RADIUS = radius_current;
      }
    }

    //#######################################################################
    //keep q_next constant
    q_next = const_cast<Configuration*>(q_next);
    Configuration *q_best = q_next;

    for(uint k = 0; k < K_samples; k++)
    {
      //obtain sample q_k, and radius radius_k
      Configuration *q_k = new Configuration(Q1);
      Q1_sampler->sampleUniformNear(q_k->state, q_next->state, SAMPLING_RADIUS);
      double d_current_to_k = DistanceQ1(q_k, q_current);
      Q1->getStateSpace()->interpolate(q_current->state, q_k->state, radius_current/d_current_to_k, q_k->state);

      double radius_next = DistanceInnerRobotToObstacle(q_k->state);

      if(verbose>1){
        std::cout << "q_k" << std::endl;
        Q1->printState(q_k->state);
        std::cout << "radius: " << radius_next << std::endl;
      }

      if(q_best->isSufficientFeasible && !q_k->isSufficientFeasible)
      {
        q_best = q_k;
        radius_best = radius_next;
      }else{
        if(radius_next > radius_best)
        {
          q_best = q_k;
          radius_best = radius_next;
        }else{
          q_k->Remove(Q1);
          continue;
        }
      }
    }
    if(!AddConfiguration(q_best, q_current, true))
    {
      return nullptr;
    }
    //#########################################################################
    //return q_next
    //#########################################################################
    return q_best;
}

void QNG::RemoveConfiguration(Configuration *q)
{
  pdf_all_configurations.remove(static_cast<PDF_Element*>(q->GetPDFElement()));
  nearest_cover->remove(q);
  nearest_vertex->remove(q);
  if(q->parent==nullptr){
    OMPL_ERROR("Trying to remove start configuration");
    exit(0);
  }
  q->Remove(Q1);
}

bool QNG::Sample(Configuration *q_random){
  if(parent == nullptr){

    double r = rng_.uniform01();
    if(!hasSolution && r<goalBias){
      q_random->state = si_->cloneState(q_goal->state);
    }else{
      if(r < goalBias + (1-goalBias)*voronoiBias){
        Q1_sampler->sampleUniform(q_random->state);
      }else{
        Configuration *q;
        if(pdf_necessary_configurations.size()>0){
          q = pdf_necessary_configurations.sample(rng_.uniform01());
          q->number_attempted_expansions++;
          pdf_necessary_configurations.update(static_cast<PDF_Element*>(q->GetPDFElement()), q->GetImportance());
        }else{
          q = pdf_all_configurations.sample(rng_.uniform01());
          q->number_attempted_expansions++;
          pdf_all_configurations.update(static_cast<PDF_Element*>(q->GetPDFElement()), q->GetImportance());
        }
        sampleHalfBallOnNeighborhoodBoundary(q_random, q);
      }
    }

    return true;
  }else{
    //Q1 = X0 x X1
    //Q0 = X0
    ob::State *stateX1 = X1->allocState();
    ob::State *stateQ0 = Q0->allocState();
    X1_sampler->sampleUniform(stateX1);
    q_random->coset = static_cast<og::QNG*>(parent)->SampleCover(stateQ0);
    if(q_random->coset == nullptr){
      OMPL_ERROR("no coset found for state");
      Q1->printState(q_random->state);
      exit(0);
    }
    mergeStates(stateQ0, stateX1, q_random->state);
    X1->freeState(stateX1);
    Q0->freeState(stateQ0);
    return true;
  }
}

QNG::Configuration* QNG::Nearest(Configuration *q) const
{
  return nearest_cover->nearest(q);
}

bool QNG::Connect(const Configuration *q_from, Configuration *q_to)
{
  double d = Distance(q_from, q_to);
  double radius = q_from->GetRadius();
  double step_size = radius/d;
  si_->getStateSpace()->interpolate(q_from->state, q_to->state, step_size, q_to->state);
  return true;
}

double QNG::Distance(const Configuration *q_from, const Configuration *q_to)
{
  if(parent == nullptr){
    return DistanceQ1(q_from, q_to);
  }else{
    if(q_to->coset == nullptr || q_from->coset == nullptr){
      return DistanceQ1(q_from, q_to);
    }
    og::QNG *QNG_parent = dynamic_cast<og::QNG*>(parent);
    return QNG_parent->DistanceCover(q_from->coset, q_to->coset)+DistanceX1(q_from, q_to);
  }
}

double QNG::DistanceQ1(const Configuration *q_from, const Configuration *q_to)
{
  return Q1->distance(q_from->state, q_to->state);
}

double QNG::DistanceX1(const Configuration *q_from, const Configuration *q_to)
{
  ob::State *stateFrom = X1->allocState();
  ob::State *stateTo = X1->allocState();
  ExtractX1Subspace(q_from->state, stateFrom);
  ExtractX1Subspace(q_to->state, stateTo);
  double d = X1->distance(stateFrom, stateTo);
  X1->freeState(stateFrom);
  X1->freeState(stateTo);
  return d;
}

double QNG::DistanceOpenNeighborhood(const Configuration *q_from, const Configuration *q_to)
{
  double d_from = q_from->GetRadius();
  double d_to = q_to->GetRadius();
  double d = si_->distance(q_from->state, q_to->state);

  //note that this is a pseudometric: invalidates second axiom of metric : d(x,y) = 0  iff x=y. But here we only have d(x,x)=0
  if(d!=d){
    // std::vector<Configuration *> vertices;
    // std::cout << std::string(80, '-') << std::endl;
    // for (auto &vertex : vertices){
    //   Q1->printState(vertex->state);
    //   std::cout << "radius " << vertex->GetRadius() << std::endl;
    // }
    std::cout << std::string(80, '-') << std::endl;
    std::cout << "NaN detected." << std::endl;
    std::cout << "d_from " << d_from << std::endl;
    std::cout << "d_to " << d_to << std::endl;
    std::cout << "d " << d << std::endl;
    std::cout << "configuration 1: " << std::endl;
    Q1->printState(q_from->state);
    std::cout << "configuration 2: " << std::endl;
    Q1->printState(q_to->state);
    std::cout << std::string(80, '-') << std::endl;

    exit(1);
  }
  double d_open_neighborhood_distance = std::max(d - d_from - d_to, 0.0); 
  return d_open_neighborhood_distance;
}

bool QNG::sampleHalfBallOnNeighborhoodBoundary(Configuration *sample, const Configuration *center)
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

  //@DEBUG
  if(epsilon_min_distance >= radius){
    std::cout << std::string(80, '-') << std::endl;
    OMPL_ERROR("neighborhood is too small to sample the boundary.");

    std::cout << std::string(80, '-') << std::endl;
    std::cout << "states currently considered:" << std::endl;
    for(uint k = 0; k < pdf_all_configurations.size(); k++){
      Configuration *q = pdf_all_configurations[k];
      std::cout << "state " << k << " with radius " << q->GetRadius() << " has values:" << std::endl;
      Q1->printState(q->state);
    }
    std::cout << std::string(80, '-') << std::endl;
    
    std::cout << "neighborhood radius: " << radius << std::endl;
    std::cout << "minimal distance to be able to sample: " << epsilon_min_distance << std::endl;
    exit(0);
  }

  //sample as long as we are inside the ball of radius epsilon_min_distance
  while(dist_q_qk <= epsilon_min_distance){
    Q1_sampler->sampleGaussian(sample->state, center->state, 1);
    dist_q_qk = si_->distance(center->state, sample->state);
  }

  //S = sample->state
  //C = center->state
  //P = center->parent->state
  si_->getStateSpace()->interpolate(center->state, sample->state, radius/dist_q_qk, sample->state);

  Configuration *q_parent = center->parent;
  if(q_parent !=nullptr)
  {
    if(DistanceQ1(q_parent, sample) < q_parent->GetRadius()){
      si_->getStateSpace()->interpolate(sample->state, center->state, 2, sample->state);
    }
  }

  return true;

}
bool QNG::sampleUniformOnNeighborhoodBoundary(Configuration *sample, const Configuration *center)
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

  //@DEBUG
  if(epsilon_min_distance >= radius){
    OMPL_ERROR("neighborhood is too small to sample the boundary.");
    std::cout << std::string(80, '-') << std::endl;
    std::cout << "states currently considered:" << std::endl;
    for(uint k = 0; k < pdf_all_configurations.size(); k++){
      Configuration *q = pdf_all_configurations[k];
      std::cout << "state " << k << " with radius " << q->GetRadius() << " has values:" << std::endl;
      Q1->printState(q->state);
    }
    std::cout << std::string(80, '-') << std::endl;
    std::cout << "neighborhood radius: " << radius << std::endl;
    std::cout << "minimal distance to be able to sample: " << epsilon_min_distance << std::endl;
    std::cout << "state:" << std::endl;
    Q1->printState(center->state);
    exit(0);
  }

  //sample as long as we are inside the ball of radius epsilon_min_distance
  while(dist_q_qk <= epsilon_min_distance){
    Q1_sampler->sampleGaussian(sample->state, center->state, 1);
    dist_q_qk = si_->distance(center->state, sample->state);
  }
  si_->getStateSpace()->interpolate(center->state, sample->state, radius/dist_q_qk, sample->state);

  return true;
}


uint QNG::GetNumberOfVertices() const
{
  return nearest_vertex->size();
}

uint QNG::GetNumberOfEdges() const
{
  std::vector<Configuration *> configs;
  if (nearest_vertex){
    nearest_vertex->list(configs);
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

void QNG::Init()
{

  checkValidity();
}


void QNG::clear()
{
  Planner::clear();
  if(nearest_cover){
    nearest_cover->clear();
  }
  if(nearest_vertex){
    nearest_vertex->clear();
  }
  hasSolution = false;
  q_start = nullptr;
  q_goal = nullptr;

  pis_.restart();
}

void QNG::freeTree( NearestNeighborsPtr nn)
{
  if(nn)
  {
    std::vector<Configuration *> configurations;
    nn->list(configurations);
    for (auto &configuration : configurations)
    {
      if (configuration->state != nullptr)
      {
        Q1->freeState(configuration->state);
      }
      delete configuration;
    }
  }
}

void QNG::CheckForSolution(ob::PathPtr &solution)
{
  Configuration *q_nearest = Nearest(q_goal);
  if(DistanceOpenNeighborhood(q_nearest, q_goal) <= 0)
  {
    if(q_goal->parent == nullptr){
      //goal has not been added before
      q_goal->parent = q_nearest;
      nearest_cover->add(q_goal);
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

void QNG::SetSubGraph( QuotientChart *sibling, uint k )
{
  QNG *rhs = dynamic_cast<og::QNG*>(sibling);
  std::cout << "old graph: " << rhs->GetNumberOfVertices() << " vertices | " << rhs->GetNumberOfEdges() << " edges."<< std::endl;

  local_chart = true;
  opt_ = rhs->opt_;
  level = rhs->GetLevel();
  startM_ = rhs->startM_;
  goalM_ = rhs->goalM_;
  number_of_paths = 0;

  nearest_cover = rhs->GetTree();
  q_start = rhs->GetStartConfiguration();
  q_goal = rhs->GetGoalConfiguration();
  pdf_necessary_configurations = rhs->GetPDFNecessaryVertices();
  pdf_all_configurations = rhs->GetPDFAllVertices();
  goalBias = rhs->GetGoalBias();

  std::cout << "new graph: " << GetNumberOfVertices() << " vertices | " << GetNumberOfEdges() << " edges."<< std::endl;
}

std::shared_ptr<ompl::NearestNeighbors<QNG::Configuration *>> QNG::GetTree() const
{
  return nearest_cover;
}
QNG::Configuration* QNG::GetStartConfiguration() const
{
  return q_start;
}
QNG::Configuration* QNG::GetGoalConfiguration() const
{
  return q_goal;
}
const QNG::PDF& QNG::GetPDFNecessaryVertices() const
{
  return pdf_necessary_configurations;
}
const QNG::PDF& QNG::GetPDFAllVertices() const
{
  return pdf_all_configurations;
}
double QNG::GetGoalBias() const
{
  return goalBias;
}


void QNG::getPlannerData(base::PlannerData &data) const
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
  if (nearest_cover){
    nearest_cover->list(vertices);
  }

  uint ctr_sufficient = 0;
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
      ctr_sufficient++;
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
    << " | siblings " << siblings.size() << " | path " << path << std::endl;

  std::cout << "               | vertices " << data.numVertices() - Nvertices << " (" << ctr_sufficient << " sufficient)" 
    << " | edges " << data.numEdges() - Nedges
    << std::endl;

  for(uint i = 0; i < siblings.size(); i++){
    siblings.at(i)->getPlannerData(data);
  }
  if(child!=nullptr) child->getPlannerData(data);
}

