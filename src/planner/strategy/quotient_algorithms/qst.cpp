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
      std::cout << "adding: " << std::endl;
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

    //#########################################################################
    OMPL_INFORM("%s: ready with %lu states already in datastructure", getName().c_str(), cover_tree->size());
    setup_ = true;
  }else{
    setup_ = false;
  }

}

bool QST::AddConfiguration(Configuration *q)
{
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
void QST::AddState(ob::State *state)
{
  Configuration *q = new Configuration(Q1, state);
  AddConfiguration(q);
}


void QST::Grow(double t)
{
  Configuration *q_random = new Configuration(si_);

  Sample(q_random);

  //reject samples which are inside another sphere
  std::vector<Configuration*> nbhs;
  cover_tree->nearestK(q_random, 2, nbhs);
  if(nbhs.size()>1){
    double d = DistanceOpenNeighborhood(q_random,nbhs.at(1));
    if(d<=threshold_clearance){
      q_random->clear(Q1);
      delete q_random;
      return;
    }
  }

  Configuration *q_nearest = Nearest(q_random);

  Connect(q_nearest, q_random);

  //#########################################################################
  //added:
  //#########################################################################
  // (1) sample only in half ball, instead of ball
  // (2) a successful expansion is an expansion where the ball does not
  // significantly shrink
  // (3) samples are rejected if they are inside the cover -> we require them to
  // lie on the boundary
  // (4) If a new sample has a NBH covering other NBHs, then remove those other
  // NBHs
  // (5) remove vertices if their importance drops below some threshold
  //#########################################################################

  const uint max_extension_steps = 1000;
  uint step = 0;

  while(AddConfiguration(q_random) && step++ < max_extension_steps)
  {
    // std::cout << "steps " << step << "/" << max_extension_steps << std::endl;
    // std::cout << "from " << std::endl;
    // Q1->printState(q_nearest->state);
    // std::cout << "to " << std::endl;
    // Q1->printState(q_random->state);

    //#########################################################################
    //remove all points which are inside our new configuration
    //#########################################################################
    double rq = q_random->GetRadius();
    vertex_tree->nearestR(q_random, rq, nbhs);
    for(uint k = 0; k < nbhs.size(); k++){
      double rk = nbhs.at(k)->GetRadius();
      double dx = DistanceQ1(q_random, nbhs.at(k));
      if(rk > rq+dx){
        if(q_nearest == nbhs.at(k))
        {
          q_nearest = q_nearest->parent;
        }
        RemoveConfiguration(nbhs.at(k));
      }
    }

    //#########################################################################
    //make sure we do not remove q_nearest!
    //#########################################################################
    q_random->parent = q_nearest;

    //#########################################################################
    //successful expansion: An expansion, where we do not shrink significantly the radii
    //#########################################################################
    if(0.8*q_nearest->GetRadius() <= q_random->GetRadius())
    {
      q_nearest->number_successful_expansions++;
      //q_random->number_successful_expansions++; //importance=1
    }else{
    }
    //#########################################################################
    //Momentum: if it is going well, continue
    //#########################################################################

    Configuration *q_next = EstimateBestNextState(q_nearest, q_random);

    //@DEBUG
    //double dn = Distance(q_nearest, q_next);
    // if( fabs(d + radius - dn) > 1e-10)
    // {
    //   std::cout << "interpolate not working correctly." << std::endl;
    //   std::cout << "nearest:" << std::endl;
    //   Q1->printState(q_nearest->state);
    //   std::cout << "random:" << std::endl;
    //   Q1->printState(q_random->state);
    //   std::cout << "radius q_random:" << radius << std::endl;
    //   std::cout << "next:" << std::endl;
    //   Q1->printState(q_next->state);
    //   std::cout << "distance q_random to q_next:" << Distance(q_random, q_next) << ". should be " << radius << std::endl;
    //   exit(0);
    // }

    Connect(q_random, q_next);

    q_nearest = q_random;
    q_random = q_next;
    q_nearest->number_attempted_expansions++;
    pdf_all_vertices.update(static_cast<PDF_Element*>(q_nearest->GetPDFElement()), q_nearest->GetImportance());
  }

  //update PDF
  pdf_all_vertices.update(static_cast<PDF_Element*>(q_nearest->GetPDFElement()), q_nearest->GetImportance());
  //if(q_nearest){

    // if(q_nearest->GetImportance() < 0.1){
    //   //do not expand exhausted vertices
    //   pdf_all_vertices.remove(static_cast<PDF_Element*>(q_nearest->GetPDFElement()));
    //   cover_tree->remove(q_nearest);
    // }
  //}
}
QST::Configuration* QST::EstimateBestNextState(const Configuration *q_last, const Configuration *q_current)
{
    Configuration *q_next = new Configuration(Q1);

    //#########################################################################
    //Strategy 1: Linear Interpolation
    //#########################################################################
    //we implement that by starting at q_nearest, interpolating to q_random, and
    //then overshooting radius/d to q_next
    //double d = Distance(q_last, q_current);
    //double radius = q_current->GetRadius();
    //Q1->getStateSpace()->interpolate(q_last->state, q_current->state, 1 + radius/d, q_next->state);
    //#########################################################################
    //Strategy 2: Try to find better state
    //#########################################################################
    double d = Distance(q_last, q_current);
    double radius = q_current->GetRadius();
    ob::State *tmp = Q1->allocState();
    ob::State *best = Q1->allocState();
    Q1->getStateSpace()->interpolate(q_last->state, q_current->state, 1 + radius/d, q_next->state);

    double d_best = fabs(DistanceRobotToObstacle(q_next->state)-radius);
    best = Q1->cloneState(tmp);
    for(uint k = 0; k < 3; k++)
    {
      Q1_sampler->sampleUniformNear(tmp, q_next->state, 0.1*radius);
      double d_next = fabs(DistanceRobotToObstacle(tmp)-radius);
      if(d_next < d_best)
      {
        best = Q1->cloneState(tmp);
        d_best = d_next;
      }
    }
    q_next->state = Q1->cloneState(best);
    q_next->SetRadius( DistanceRobotToObstacle(best) );

    Q1->freeState(best);
    Q1->freeState(tmp);

    //#########################################################################
    //return q_next
    //#########################################################################
    return q_next;
}

void QST::RemoveConfiguration(Configuration *q)
{
  pdf_all_vertices.remove(static_cast<PDF_Element*>(q->GetPDFElement()));
  cover_tree->remove(q);
  vertex_tree->remove(q);
  q->clear(Q1);
  delete q;
}

bool QST::Sample(Configuration *q_random){
  if(parent == nullptr){

    double r = rng_.uniform01();
    if(r<goalBias){
      q_random->state = si_->cloneState(q_goal->state);
    }else{
      Configuration *q = pdf_all_vertices.sample(rng_.uniform01());
      q->number_attempted_expansions++;
      pdf_all_vertices.update(static_cast<PDF_Element*>(q->GetPDFElement()), q->GetImportance());

      if(q->GetImportance() >= 0.5){
        sampleHalfBallOnNeighborhoodBoundary(q_random, q);
      }else{
        sampleUniformOnNeighborhoodBoundary(q_random, q);
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

void QST::Connect(const Configuration *q_from, Configuration *q_to)
{
  double d = Distance(q_from, q_to);
  double radius = q_from->GetRadius();
  si_->getStateSpace()->interpolate(q_from->state, q_to->state, radius / d, q_to->state);
}

double QST::Distance(const Configuration *q_from, const Configuration *q_to)
{
  if(parent == nullptr){
    return DistanceQ1(q_from, q_to);
  }else{
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
  std::cout << std::string(80, '-') << std::endl;
  std::cout << "from" << std::endl;
  Q1->printState(q_from->state);
  std::cout << "to" << std::endl;
  Q1->printState(q_to->state);



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
  std::cout << "path1: " << path_1.size() << std::endl;
  for(uint k = 0; k < path_1.size(); k++){
    Q1->printState(path_1.at(k)->state);
  }
  std::cout << "path2: " << path_2.size() << std::endl;
  for(uint k = 0; k < path_2.size(); k++){
    Q1->printState(path_2.at(k)->state);
  }
  std::cout << std::string(80, '-') << std::endl;
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

  //std::cout << "path: " << path->getStateCount() << std::endl;
  //for(uint k = 0; k < path->getStateCount(); k++){
  //  Q1->printState(path->getState(k));
  //}

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
    std::cout << "NaN detected." << std::endl;
    std::cout << "d_from " << d_from << std::endl;
    std::cout << "d_to " << d_to << std::endl;
    std::cout << "d " << d << std::endl;
    std::cout << "configuration 1: " << std::endl;
    Q1->printState(q_from->state);
    std::cout << "configuration 2: " << std::endl;
    Q1->printState(q_to->state);

    std::vector<Configuration *> vertices;
    if (cover_tree){
      cover_tree->list(vertices);
    }
    for (auto &vertex : vertices){
      Q1->printState(vertex->state);
      std::cout << "vertex " << vertex->GetRadius() << std::endl;
    }
    exit(0);
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
  double epsilon_min_distance = 1e-10;

  //@DEBUG
  if(epsilon_min_distance >= radius){
    OMPL_ERROR("neighborhood is too small to sample the boundary.");
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
  double epsilon_min_distance = 1e-10;

  //@DEBUG
  if(epsilon_min_distance >= radius){
    OMPL_ERROR("neighborhood is too small to sample the boundary.");
    std::cout << "neighborhood radius: " << radius << std::endl;
    std::cout << "minimal distance to be able to sample: " << epsilon_min_distance << std::endl;
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

