#include "common.h"
#include "qst.h"
#include "elements/plannerdata_vertex_annotated.h"
#include "planner/cspace/validitychecker/validity_checker_ompl.h"
#include <limits>


using namespace ompl::geometric;

QST::QST(const base::SpaceInformationPtr &si, Quotient *parent ): BaseT(si, parent)
{
  setName("QST"+std::to_string(id));
  if (!cover_tree){
    cover_tree.reset(tools::SelfConfig::getDefaultNearestNeighbors<Configuration *>(this));
    cover_tree->setDistanceFunction([this](const Configuration *a, const Configuration *b)
                             {
                                return DistanceOpenNeighborhood(a,b);
                              });
  }
  if (!vertex_tree){
    vertex_tree.reset(tools::SelfConfig::getDefaultNearestNeighbors<Configuration *>(this));
    vertex_tree->setDistanceFunction([this](const Configuration *a, const Configuration *b)
                             {
                                return DistanceQ1(a,b);
                              });
  }
}

QST::~QST(void)
{
}

void QST::setup(void)
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
      q_start = new Configuration(Q1, state);
      bool added = AddConfiguration(q_start);
      if(!added)
      {
        OMPL_ERROR("Could not add start state.");
        exit(0);
      }
      if(parent != nullptr)
      {
        q_start->coset = static_cast<og::QST*>(parent)->q_start;
      }
    }else{
      OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
      exit(0);
    }

    //#########################################################################
    //Adding goal configuration
    //#########################################################################
    if(const ob::State *state = pis_.nextGoal()){
      q_goal = new Configuration(si_, state);
      if(parent != nullptr)
      {
        q_goal->coset = static_cast<og::QST*>(parent)->q_goal;
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
    OMPL_INFORM("%s: ready with %lu states already in datastructure", getName().c_str(), cover_tree->size());
    setup_ = true;
  }else{
    setup_ = false;
  }

}

bool QST::AddConfiguration(Configuration *q, Configuration *q_parent, bool allowInsideCover)
{
  if(!allowInsideCover){
    if(IsSampleInsideCover(q)){
      q->clear(Q1);
      delete q;
      return false;
    }
  }

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
      q->clear(Q1);
      delete q;
      return false;
    }
    q->parent = q_parent;
    if(q_parent != nullptr) q_parent->children.push_back(q);

    //sample lies outside our cover, but some samples might now be contained by
    //q
    RemoveCoveredSamples(q);

    //add to structures
    if(!q->isSufficientFeasible){
      pdf_necessary_vertices.add(q, q->GetRadius());
    }
    PDF_Element *q_element = pdf_all_vertices.add(q, q->GetImportance());
    q->SetPDFElement(q_element);
    cover_tree->add(q);
    vertex_tree->add(q);
  }else{
    q->clear(Q1);
    delete q;
    return false;
  }

  return true;
}


//if new sample is inside the current cover, return true
//we check the two nearest points. only one of them can be a parent, ignore that
//one, and check distance of the second one.

bool QST::IsSampleInsideCover(Configuration *q)
{
  std::vector<Configuration*> neighbors;

  cover_tree->nearestK(q, 2, neighbors);

  for(uint k = 0; k < neighbors.size(); k++)
  {
    Configuration *qn = neighbors.at(k);
    //the configuration is allowed to be on the boundary of the parent
    if(qn != q->parent){
      double dqn = DistanceQ1(qn, q);
      double radius_n = qn->GetRadius();
      if( dqn < radius_n ){
        //std::cout << "distance rejected: " << dqn << " < " << radius_n << std::endl;
        return true;
      }
    }
  }
  return false;
}

//If the neighborhood of q is a superset of any other neighborhood, then delete
//the other neighborhood, and rewire the tree.
void QST::RemoveCoveredSamples(Configuration *q)
{
  double radius_q = q->GetRadius();
  std::vector<Configuration*> neighbors;

  //get all vertices inside open set of q
  vertex_tree->nearestR(q, radius_q, neighbors);

  for(uint k = 0; k < neighbors.size(); k++){

    Configuration *qn = neighbors.at(k);

    if(qn->parent == nullptr) continue; //start configuration
    //if(qn == q->parent) continue; //parent configuration

    double distance_q_qn = DistanceQ1(q, qn);
    double radius_k = qn->GetRadius();

    //if(radius_q > distance_q_qn){
    if(radius_q > radius_k+distance_q_qn){
      RemoveConfiguration(qn);
    }
  }
}

void QST::Grow(double t)
{
  if(saturated) return;

  Configuration *q_random = new Configuration(si_);

  Sample(q_random);


  Configuration *q_nearest = Nearest(q_random);
  if(q_nearest->state == q_random->state){
    OMPL_ERROR("sampled and nearest states are equivalent.");
    std::cout << "sampled state" << std::endl;
    Q1->printState(q_random->state);
    std::cout << "nearest state" << std::endl;
    Q1->printState(q_nearest->state);
    exit(0);
  }

  if(!Connect(q_nearest, q_random)){
    q_random->clear(Q1);
    delete q_random;
    return;
  }

  //  q_nearest  ---->  q_random  ---->  q_next
  std::cout << std::string(80, '-') << std::endl;
  std::cout << "new iteration" << std::endl;
  std::cout << std::string(80, '-') << std::endl;
  if(AddConfiguration(q_random, q_nearest)){
    const uint max_extension_steps = 1000;
    uint step = 0;
    while(step++ < max_extension_steps)
    {
      //#########################################################################
      //Momentum: if it is going well, continue
      //#########################################################################
      q_random->number_attempted_expansions++;

      std::cout << std::string(80, '-') << std::endl;
      std::cout << "step: " << step << std::endl;
      std::cout << "q_random:" << std::endl;
      Q1->printState(q_random->state);
      pdf_all_vertices.update(static_cast<PDF_Element*>(q_random->GetPDFElement()), q_random->GetImportance());

      Configuration *q_next = EstimateBestNextState(q_nearest, q_random);

      if(q_next == nullptr)
      {
        std::cout << "q_next is nullptr" << std::endl;
        break;
      }
      std::cout << "q_next:" << std::endl;
      Q1->printState(q_next->state);
      //note that q_random might have been removed if the neighborhood is a
      //proper subset of the neighborhood of q_next
      if(q_random == nullptr){
        Connect(q_nearest, q_next);
      }else{
        std::cout << "q_random:" << std::endl;
        Q1->printState(q_random->state);
        Connect(q_random, q_next);
        pdf_all_vertices.update(static_cast<PDF_Element*>(q_random->GetPDFElement()), q_random->GetImportance());
        q_nearest = q_random;
      }
      q_random = q_next;
    }
  }

  //update PDF
  if(q_nearest != nullptr){
    pdf_all_vertices.update(static_cast<PDF_Element*>(q_nearest->GetPDFElement()), q_nearest->GetImportance());
  }
  //if(q_nearest){

    // if(q_nearest->GetImportance() < 0.1){
    //   //do not expand exhausted vertices
    //   pdf_all_vertices.remove(static_cast<PDF_Element*>(q_nearest->GetPDFElement()));
    //   cover_tree->remove(q_nearest);
    // }
  //}
}
QST::Configuration* QST::EstimateBestNextState(Configuration *q_last, Configuration *q_current)
{
    uint K_samples = 3; //how many samples to test for best direction (depends maybe also on radius)

    Configuration *q_next = new Configuration(Q1);

    std::cout << "from" << std::endl;
    Q1->printState(q_last->state);
    std::cout << "to" << std::endl;
    Q1->printState(q_current->state);

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
    std::cout << "next" << std::endl;
    Q1->printState(q_next->state);
    std::cout << "radius " << radius_best << std::endl;

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

      std::cout << "q_k" << std::endl;
      Q1->printState(q_k->state);
      std::cout << "radius: " << radius_next << std::endl;

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
          q_k->clear(Q1);
          delete q_k;
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

void QST::RemoveConfiguration(Configuration *q)
{
  pdf_all_vertices.remove(static_cast<PDF_Element*>(q->GetPDFElement()));
  cover_tree->remove(q);
  vertex_tree->remove(q);
  if(q->parent==nullptr){
    OMPL_ERROR("Trying to remove start configuration");
    exit(0);
  }
  //if q is not a leaf node, we need to rewire all its children
  for(uint k = 0; k < q->children.size(); k++){
    Configuration *qc = q->children.at(k);
    qc->parent = q->parent;
  }
  q->children.clear();

  q->clear(Q1);
  delete q;
}

bool QST::Sample(Configuration *q_random){
  if(parent == nullptr){

    double r = rng_.uniform01();
    if(!hasSolution && r<goalBias){
      q_random->state = si_->cloneState(q_goal->state);
    }else{
      if(r < goalBias + (1-goalBias)*voronoiBias){
        Q1_sampler->sampleUniform(q_random->state);
      }else{
        Configuration *q;
        if(pdf_necessary_vertices.size()>0){
          q = pdf_necessary_vertices.sample(rng_.uniform01());
          q->number_attempted_expansions++;
          pdf_necessary_vertices.update(static_cast<PDF_Element*>(q->GetPDFElement()), q->GetImportance());
        }else{
          q = pdf_all_vertices.sample(rng_.uniform01());
          q->number_attempted_expansions++;
          pdf_all_vertices.update(static_cast<PDF_Element*>(q->GetPDFElement()), q->GetImportance());
        }
        sampleHalfBallOnNeighborhoodBoundary(q_random, q);
        //if(q->GetImportance() >= 0.5){
        //  sampleHalfBallOnNeighborhoodBoundary(q_random, q);
        //}else{
        //  sampleUniformOnNeighborhoodBoundary(q_random, q);
        //}
      }
    }

    return true;
  }else{
    //Q1 = X0 x X1
    //Q0 = X0
    ob::State *stateX1 = X1->allocState();
    ob::State *stateQ0 = Q0->allocState();
    X1_sampler->sampleUniform(stateX1);
    q_random->coset = static_cast<og::QST*>(parent)->SampleTree(stateQ0);
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
og::QST::Configuration* QST::SampleTree(ob::State *q_random_graph)
{
  Configuration *q = pdf_necessary_vertices.sample(rng_.uniform01());
  double d = q->openNeighborhoodRadius;
  Q1_sampler->sampleUniformNear(q_random_graph, q_random_graph, d);
  return q;
}


QST::Configuration* QST::Nearest(Configuration *q) const
{
  return cover_tree->nearest(q);
}

bool QST::Connect(const Configuration *q_from, Configuration *q_to)
{
  try{
    double d = Distance(q_from, q_to);
    double radius = q_from->GetRadius();
    double step_size = radius/d;
    si_->getStateSpace()->interpolate(q_from->state, q_to->state, step_size, q_to->state);
  }catch (const std::exception& e){
    std::cout << e.what();
  }
  return true;
}

double QST::Distance(const Configuration *q_from, const Configuration *q_to)
{
  if(parent == nullptr){
    return DistanceQ1(q_from, q_to);
  }else{
    if(q_to->coset == nullptr || q_from->coset == nullptr){
      return DistanceQ1(q_from, q_to);
    }
    og::QST *qst_parent = dynamic_cast<og::QST*>(parent);
    return qst_parent->DistanceTree(q_from->coset, q_to->coset)+DistanceX1(q_from, q_to);
  }
}

double QST::DistanceQ1(const Configuration *q_from, const Configuration *q_to)
{
  return Q1->distance(q_from->state, q_to->state);
}

double QST::DistanceX1(const Configuration *q_from, const Configuration *q_to)
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

double QST::DistanceTree(const Configuration *q_from, const Configuration *q_to)
{
  //assume: q_from, q_to are vertices from the current tree. 
  //distancetree: euclidean distance along the unique path from q_from to q_to
  //###########################################################################
  //(1) compute path q_from to q_to
  //###########################################################################
  //
  //       root        |
  //       /           |
  //       |           |
  //      / \          |
  //     /   \         |
  //  q_to   q_from    |
  //
  //Traverse upwards from q_from to root, store ptrs to configurations in path_1
  //Traverse upwards from q_to to root, store ptrs to configurations in path_2
  //Then traverse downward until the intersection is met. Call the intersection
  //it2, call the first node towards q_from it1
  //
  //      it2          |
  //     / \           |
  //    .  it1         |
  //    /    \         |
  //  q_to   q_from    |
  //  
  // Then start at it1 and parse the ompl states from the configurations into 
  // 'path'. Once done, reverse this path, so that it starts at q_from and runs to
  // it1. Then add all states starting at it2 and parsing down to q_to.
  // The length of the resulting path is the unique shortest path on the tree
  // between q_to and q_from

  //###########################################################################
  //DEBUG
  //###########################################################################

  if(verbose>1){
    std::cout << std::string(80, '-') << std::endl;
    std::cout << "from" << std::endl;
    Q1->printState(q_from->state);
    std::cout << "to" << std::endl;
    Q1->printState(q_to->state);
  }

  std::vector<const Configuration *> path_1;
  const Configuration *q_ptr1 = q_from;

  while(q_ptr1->parent != nullptr)
  {
    path_1.push_back(q_ptr1);
    q_ptr1 = q_ptr1->parent;
  }
  path_1.push_back(q_ptr1);

  std::vector<const Configuration *> path_2;
  const Configuration *q_ptr2 = q_to;
  while(q_ptr2->parent != nullptr)
  {
    path_2.push_back(q_ptr2);
    q_ptr2 = q_ptr2->parent;
  }
  path_2.push_back(q_ptr2);

  //###########################################################################
  //DEBUG
  //###########################################################################
  if(verbose > 1){
    std::cout << "path1: " << path_1.size() << std::endl;
    for(uint k = 0; k < path_1.size(); k++){
      Q1->printState(path_1.at(k)->state);
    }
    std::cout << "path2: " << path_2.size() << std::endl;
    for(uint k = 0; k < path_2.size(); k++){
      Q1->printState(path_2.at(k)->state);
    }
    std::cout << std::string(80, '-') << std::endl;
  }
  //###########################################################################

  std::vector<const Configuration*>::iterator it1 = path_1.end()-1;
  std::vector<const Configuration*>::iterator it2 = path_2.end()-1;

  
  while(*it1 == *it2 && (it1 != path_1.begin() || it2 != path_2.begin()))
  {
    if(it1 != path_1.begin()) it1--;
    if(it2 != path_2.begin()) it2--;
  }

  //intersection node is different from q_from and q_to
  if(it1 != path_1.begin() && it2 != path_2.begin())
  {
    //std::cout << "unique intersection node" << std::endl;
    it2++;
  }


  //measure distance q_from to it1, it1 to it2, and it2 to q_to
  //q_from to it1
  auto path(std::make_shared<PathGeometric>(Q1));
  while(it1 != path_1.begin())
  {
    path->append((*it1)->state);
    it1--;
  }
  path->append((*it1)->state);
  path->reverse();

  while(it2 != path_2.begin())
  {
    path->append((*it2)->state);
    it2--;
  }
  path->append((*it2)->state);

  if(verbose>1){
    std::cout << "final path: " << path->getStateCount() << std::endl;
    for(uint k = 0; k < path->getStateCount(); k++){
      Q1->printState(path->getState(k));
    }
  }

  //###########################################################################
  //(2) estimate distance along path
  //###########################################################################
  // std::cout << path->length() << std::endl;
  // exit(0);
  return path->length();
}

double QST::DistanceOpenNeighborhood(const Configuration *q_from, const Configuration *q_to)
{
  double d_from = q_from->GetRadius();
  double d_to = q_to->GetRadius();
  double d = si_->distance(q_from->state, q_to->state);

  //note that this is a pseudometric: invalidates second axiom of metric : d(x,y) = 0  iff x=y. But here we only have d(x,x)=0
  if(d!=d){
    // std::vector<Configuration *> vertices;
    // if (cover_tree){
    //   cover_tree->list(vertices);
    // }
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

bool QST::sampleHalfBallOnNeighborhoodBoundary(Configuration *sample, const Configuration *center)
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
    for(uint k = 0; k < pdf_all_vertices.size(); k++){
      Configuration *q = pdf_all_vertices[k];
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

  //@DEBUG
  if(epsilon_min_distance >= radius){
    OMPL_ERROR("neighborhood is too small to sample the boundary.");
    std::cout << std::string(80, '-') << std::endl;
    std::cout << "states currently considered:" << std::endl;
    for(uint k = 0; k < pdf_all_vertices.size(); k++){
      Configuration *q = pdf_all_vertices[k];
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
  freeTree(cover_tree);
  if(cover_tree){
    cover_tree->clear();
  }
  freeTree(vertex_tree);
  if(vertex_tree){
    vertex_tree->clear();
  }
  hasSolution = false;
  q_start = nullptr;
  q_goal = nullptr;

  pis_.restart();
}

void QST::freeTree( NearestNeighborsPtr nn)
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

void QST::CheckForSolution(ob::PathPtr &solution)
{
  Configuration *q_nearest = Nearest(q_goal);
  if(DistanceOpenNeighborhood(q_nearest, q_goal) <= 0)
  {
    if(q_goal->parent == nullptr){
      //goal has not been added before
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

void QST::SetSubGraph( QuotientChart *sibling, uint k )
{
  QST *rhs = dynamic_cast<og::QST*>(sibling);
  std::cout << "old graph: " << rhs->GetNumberOfVertices() << " vertices | " << rhs->GetNumberOfEdges() << " edges."<< std::endl;

  local_chart = true;
  opt_ = rhs->opt_;
  level = rhs->GetLevel();
  startM_ = rhs->startM_;
  goalM_ = rhs->goalM_;
  number_of_paths = 0;

  cover_tree = rhs->GetTree();
  q_start = rhs->GetStartConfiguration();
  q_goal = rhs->GetGoalConfiguration();
  pdf_necessary_vertices = rhs->GetPDFNecessaryVertices();
  pdf_all_vertices = rhs->GetPDFAllVertices();
  goalBias = rhs->GetGoalBias();

  std::cout << "new graph: " << GetNumberOfVertices() << " vertices | " << GetNumberOfEdges() << " edges."<< std::endl;
}

std::shared_ptr<ompl::NearestNeighbors<QST::Configuration *>> QST::GetTree() const
{
  return cover_tree;
}
QST::Configuration* QST::GetStartConfiguration() const
{
  return q_start;
}
QST::Configuration* QST::GetGoalConfiguration() const
{
  return q_goal;
}
const QST::PDF& QST::GetPDFNecessaryVertices() const
{
  return pdf_necessary_vertices;
}
const QST::PDF& QST::GetPDFAllVertices() const
{
  return pdf_all_vertices;
}
double QST::GetGoalBias() const
{
  return goalBias;
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

