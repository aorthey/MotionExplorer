#include "common.h"
#include "gui/common.h"
#include "qng_goal_directed.h"

using namespace ompl::geometric;

//############################################################################
// Setup
//############################################################################
QNGGoalDirected::QNGGoalDirected(const base::SpaceInformationPtr &si, Quotient *parent ): BaseT(si, parent)
{
  setName("QNGGoalDirected"+std::to_string(id));
  progressMadeTowardsGoal = true;
}

QNGGoalDirected::~QNGGoalDirected(void)
{
}

void QNGGoalDirected::setup()
{
  BaseT::setup();
}

void QNGGoalDirected::clear()
{
  pdf_connectivity_configurations.clear();
  while(!configurations_sorted_by_nearest_to_goal.empty()) 
  {
    configurations_sorted_by_nearest_to_goal.pop();
  }

  nearest_to_goal_has_changed = true;
  BaseT::clear();
}


//############################################################################
// Grow Functions
//############################################################################
void QNGGoalDirected::Grow(double t)
{
  if(saturated) return;
  ob::PlannerTerminationCondition ptc( ob::timedPlannerTerminationCondition(t) );

  int N = boost::num_edges(graph);
  if(!hasSolution){
    GrowWithoutSolution(ptc);
  }else{
    exit(0);
    //GrowWithSolution(ptc);
  }
  int M = boost::num_edges(graph);
  if( abs(N-M) > 100 ){
    std::cout << "increase of edges from " << N << " to " << M << std::endl;
    //exit(0);
  }
}

void QNGGoalDirected::GrowWithoutSolution(ob::PlannerTerminationCondition &ptc)
{
  if(nearest_to_goal_has_changed){
    q_lodestar = q_goal;
    nearest_to_goal_has_changed = false;
  }
  StepTowardsLodestar();
}

void QNGGoalDirected::GrowWithSolution(ob::PlannerTerminationCondition &ptc)
{
  double r = rng_.uniform01();
  if(r <= rewireBias){
    RewireCover(ptc);
  }else{
    StepTowardsLodestar();
  }
}
bool QNGGoalDirected::GetSolution(ob::PathPtr &solution)
{
  //will be called only once
  bool result = BaseT::GetSolution(solution);
  if(result){
    //update lodestar
    q_lodestar = new Configuration(Q1);
    SampleUniform(q_lodestar);
    if(level>0) QuotientCover::Print(q_lodestar, false);
  }
  return result;
}

//############################################################################
// Step Functions
//############################################################################
void QNGGoalDirected::StepTowardsLodestar()
{
  Configuration *q_nearest = Nearest(q_lodestar);
  bool madeProgress = StepTowards(q_nearest, q_lodestar);
  if(!madeProgress){
    //change lodestar
    q_lodestar = new Configuration(Q1);
    SampleUniform(q_lodestar);
    if(level>0) QuotientCover::Print(q_lodestar, false);
  }
}


//Create a bunch of children configurations into the direction of q_next. then
//choose the child with the largest neighborhood and add it. The remaining
//children are added to the priority queue for later extensions
bool QNGGoalDirected::StepTowards(Configuration *q_from, Configuration *q_next)
{
  const double distanceFromTo = DistanceNeighborhoodNeighborhood(q_from, q_next);
  if(distanceFromTo < 1e-10){
    return false;
  }
  std::vector<Configuration*> q_children = GenerateCandidateDirections(q_from, q_next);

  if(q_children.empty()){
    return false;
  }

  uint idx_best = GetLargestNeighborhoodIndex(q_children);

  Configuration *q_best = q_children.at(idx_best);
  AddConfigurationToCover(q_best);

  //add the smaller children to the priority_configurations to be extended at
  //a later stage if required
  for(uint k = 0; k < q_children.size(); k++)
  {
    Configuration *q_k = q_children.at(k);
    if(k!=idx_best){
      priority_configurations.push(q_k);
    }
  }
  return true;
}

bool QNGGoalDirected::ConfigurationHasNeighborhoodLargerThan(Configuration *q, double radius)
{
  return q->GetRadius() >= radius;
}

std::vector<QuotientCover::Configuration*> QNGGoalDirected::GenerateCandidateDirections(Configuration *q_from, Configuration *q_next)
{
  std::vector<Configuration*> q_children;

  //            \                                           |
  //             \q_k                                       |
  //              \                                         |
  // q_from        |q_proj                      q_next      |
  //              /                                         |
  //             /                                          |
  //############################################################################
  //Case(1): Next projected onto neighborhood has larger radius => expand
  //directly, larger is always better
  //############################################################################
  Configuration *q_proj = new Configuration(Q1);

  const double radius_from = q_from->GetRadius();
  Connect(q_from, q_next, q_proj);
  q_proj->parent_neighbor = q_from;

  if(!ComputeNeighborhood(q_proj)) return q_children;

  if(ConfigurationHasNeighborhoodLargerThan(q_proj, radius_from))
  {
    q_children.push_back(q_proj);
    return q_children;
  }

  //############################################################################
  //Case(2): Next projected is smaller. Try continuing in the direction of
  //parent (added momentum)
  //############################################################################
  Configuration *q_sample_direction = q_proj;

  if(q_from->parent_neighbor != nullptr){
    Configuration *q_momentum = new Configuration(Q1);

    double radius_parent = q_from->parent_neighbor->GetRadius();
    double step = (radius_from+radius_parent)/radius_parent;
    Interpolate(q_from->parent_neighbor, q_from, step, q_momentum);

    //CheckConfigurationIsOnBoundary(q_momentum, q_from);
    q_momentum->parent_neighbor = q_from;
    //############################################################################

    if(ComputeNeighborhood(q_momentum)){
      q_children.push_back(q_momentum);
      if(ConfigurationHasNeighborhoodLargerThan(q_momentum, radius_from))
      {
        return q_children;
      }
    }

    if(q_momentum->GetRadius() > q_proj->GetRadius())
    {
      q_sample_direction = q_momentum;
    }
  }

  //############################################################################
  //Case(3): Neither proj nor momentum configuration are better. Try random
  //sampling
  //############################################################################

  for(uint k = 0; k < NUMBER_OF_EXPANSION_SAMPLES; k++){
    Configuration *q_k = new Configuration(Q1);

    Q1_sampler->sampleUniformNear(q_k->state, q_sample_direction->state, 0.5*radius_from);

    const double d_from_to_k = DistanceConfigurationConfiguration(q_from, q_k);

    double step_size = radius_from/d_from_to_k;
    Q1->getStateSpace()->interpolate(q_from->state, q_k->state, step_size, q_k->state);

    // std::cout << "radius q_from: " << q_from->GetRadius() << std::endl;
    // std::cout << "d_from_to_k  : " << d_from_to_k << std::endl;
    // std::cout << "step         : " << step_size << std::endl;
    // std::cout << "d_from_to_k  (after proj): " << DistanceQ1(q_from, q_k) << std::endl;
    // double d_outcome = DistanceConfigurationConfiguration(q_from, q_k);
    // std::cout << "d_from_to_k  (graph metric): " << d_outcome << std::endl;

    //PROBLEM: q_k does not have a coset after sampling. Therefore we utilize
    //internally DistanceQ1. After Computeneighborhood, the coset is set, and we
    //can actually utilize the graph metric. However, under the graph metric,
    //the distance changes and it is not on the boundary anymore. Not sure how
    //to resolve
    //
    //
    if(ComputeNeighborhood(q_k))
    {
      // d_outcome = DistanceConfigurationConfiguration(q_from, q_k);
      // std::cout << "d_from_to_k  (graph metric): " << d_outcome << std::endl;
      //############################################################################
      //if(verbose>2) CheckConfigurationIsOnBoundary(q_k, q_from);
      q_k->parent_neighbor = q_from;
      //############################################################################

      q_children.push_back(q_k);
    }
  }
  return q_children;
}

//############################################################################
// Misc Functions
//############################################################################

double QNGGoalDirected::ValueConnectivity(Configuration *q)
{
  Vertex v = get(indexToVertex, q->index);
  double d_alpha = std::pow(2.0,boost::degree(v, graph));
  //double d_alpha = boost::degree(v, graph)+1;
  double d_connectivity = q->GetRadius()/d_alpha;
  //double d_connectivity = q->GetRadius();
  return d_connectivity;
}

void QNGGoalDirected::RewireCover(ob::PlannerTerminationCondition &ptc)
{
  Configuration *q = pdf_connectivity_configurations.sample(rng_.uniform01());

  //find all vertices which intersect NBH, then check if they have an edge in
  //common. Then add one if they don't.

  //RewireConfiguration(q);
  //for(uint k = 0; k < q_neighbors.size(); k++){
  //  Configuration *qk = q_neighbors.at(k);
  //  if(!EdgeExists(q, qk)){
  //    pdf_connectivity_configurations.update(static_cast<PDF_Element*>(qk->GetConnectivityPDFElement()), ValueConnectivity(qk));
  //    AddEdge(q, qk);
  //  }
  //}

  //add one more edges
  std::vector<Configuration*> neighbors;
  Vertex v = get(indexToVertex, q->index);
  uint K = boost::degree(v, graph)+1;
  nearest_neighborhood->nearestK(q, K, neighbors);

  if(neighbors.size()>=K){
    Configuration *qn = neighbors.at(K-1);
    double dn = DistanceNeighborhoodNeighborhood(q, qn);
    if(dn <= 1e-10){
      AddEdge(q, qn);
      pdf_connectivity_configurations.update(static_cast<PDF_Element*>(qn->GetConnectivityPDFElement()), ValueConnectivity(qn));
      pdf_connectivity_configurations.update(static_cast<PDF_Element*>(q->GetConnectivityPDFElement()), ValueConnectivity(q));
    }
  }
}

QuotientCover::Vertex QNGGoalDirected::AddConfigurationToCover(Configuration *q)
{
  QuotientCover::Vertex v = BaseT::AddConfigurationToCover(q);

  PDF_Element *q_element = pdf_connectivity_configurations.add(q, ValueConnectivity(q));
  q->SetConnectivityPDFElement(q_element);

  Configuration *q_nearest = nullptr;
  if(!configurations_sorted_by_nearest_to_goal.empty()){
    q_nearest = configurations_sorted_by_nearest_to_goal.top();
  }

  configurations_sorted_by_nearest_to_goal.push(q);
  if(q_nearest != configurations_sorted_by_nearest_to_goal.top()){
    //change in nearest towards goal. This should trigger the step towards goal
    //method. This removes the goalBias
    nearest_to_goal_has_changed = true;
  }

  return v;
}

