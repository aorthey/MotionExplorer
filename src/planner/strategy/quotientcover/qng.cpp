#include "common.h"
#include "qng.h"
#include "elements/plannerdata_vertex_annotated.h"
#include "planner/cspace/validitychecker/validity_checker_ompl.h"
#include <limits>
#include <boost/graph/astar_search.hpp>
#include <boost/graph/incremental_components.hpp>
#include <boost/property_map/vector_property_map.hpp>
#include <boost/property_map/transform_value_property_map.hpp>
#include <boost/foreach.hpp>

#define foreach BOOST_FOREACH

using namespace ompl::geometric;

QNG::QNG(const base::SpaceInformationPtr &si, Quotient *parent ): BaseT(si, parent)
{
  setName("QNG"+std::to_string(id));
  if(importanceSamplingBias>0.9)
  {
  }
}

QNG::~QNG(void)
{
}
void QNG::clear()
{
  BaseT::clear();
  while(!priority_configurations.empty()) 
  {
    priority_configurations.pop();
  }
  fast_track_configurations.clear();
  firstRun = true;
}

void QNG::setup(void)
{
  BaseT::setup();
}
void QNG::Grow(double t)
{
  ob::PlannerTerminationCondition ptc( ob::timedPlannerTerminationCondition(t) );
  //#########################################################################
  //Do not grow the cover if it is saturated, i.e. it cannot be expanded anymore
  // --- we have successfully computed Q1_{free}, the free space of Q1
  //#########################################################################
  if(saturated) return;

  //#########################################################################
  //Three different strategies: 
  //(1) Expand fast track configurations (momentum-like expansions)
  //(2) Expand a random important node
  //(3) Expand using voronoi bias and goal bias
  //#########################################################################
  //Rational for fast track: If we make progress with one node, then try to
  //stick to this node and expand as long as we are stopping making progress.
  //(This allows us to quickly traverse a narrow passage, or to quickly expand
  //into a free open space. It breaks down, however, when we try to find
  //the entrance to a narrow passage. This is why we use it in combination with
  //other strategies)
  //Everytime we expanded a fast track node, we delete it from the fast_track
  //nodes. Then we add its best child to the fast_track, but only if the child
  //is not significantly worse than the previous fast_track node (i.e. we made
  //progress)
  //#########################################################################
  //Rational for random importance sampling: Voronoi bias can be misleading,
  //especially if the solution is in the direction of low voronoi bias.
  //Therefore, we should try to go into a random direction starting at the node.
  //Note that voronoi bias is not choosing an equally distributed direction,
  //i.e. it imposes a non-uniform probability distribution on the tangent space
  //of the node. Random importance sampling uses a uniform probability
  //distribution instead.

  Configuration *q = nullptr;
  if(firstRun)
  {
    q = q_start;
    firstRun = false;
  }else{
    if(priority_configurations.empty()) return;
    //get last element and remove from pq
    q = priority_configurations.top();
    priority_configurations.pop();
    if(IsConfigurationInsideCover(q)){
      if(verbose>0) std::cout << "removing neighborhood with radius " << q->GetRadius() << std::endl;
      if(verbose>0) Q1->printState(q->state);
      q->Remove(Q1);
      q=nullptr;
      return;
    }
    AddConfigurationToCoverWithoutAddingEdges(q);
  }

  //strategy: e

  if(verbose>0) std::cout << std::string(80, '-') << std::endl;
  if(verbose>0) std::cout << "expanding neighborhood with radius " << q->GetRadius() << std::endl;
  Q1->printState(q->state);

  //get all children (samples on the boundary of the neighborhood)
  std::vector<Configuration*> q_children = ExpandNeighborhood(q);

  if(verbose>0) std::cout << "children:" << std::endl;
  for(uint k = 0; k < q_children.size(); k++){
    if(verbose>0) Q1->printState(q_children.at(k)->state);
    if(verbose>0) std::cout << "radius: " << q_children.at(k)->GetRadius() << std::endl;
    priority_configurations.push(q_children.at(k));
  }
  if(verbose>0) std::cout << "size priority_queue:" << priority_configurations.size() << std::endl;




  double r = rng_.uniform01();
  if(r < importanceSamplingBias){
    Configuration *q_important = pdf_all_configurations.sample(rng_.uniform01());
    q_important->number_attempted_expansions++;
    pdf_all_configurations.update(static_cast<PDF_Element*>(q_important->GetPDFElement()), q_important->GetImportance());

    std::vector<Configuration*> q_children = ExpandNeighborhood(q_important);

    for(uint k = 0; k < q_children.size(); k++){
      priority_configurations.push(q_children.at(k));
    }
  }else{
    //classical RRT-like expansion with goal-bias
    Configuration *q_random = SampleCoverBoundaryValid(ptc);
    if(q_random == nullptr) return;
    priority_configurations.push(q_random);
  }
  
}

std::vector<QNG::Configuration*> QNG::ExpandNeighborhood(Configuration *q)
{
  std::vector<Configuration*> q_children;
  if(q->parent_neighbor == nullptr)
  {
    //if no parent neighbor is available, just sample on the neighborhood boundary
    for(uint k = 0; k < NUMBER_OF_EXPANSION_SAMPLES; k++){
      Configuration *q_random = new Configuration(Q1);
      SampleNeighborhoodBoundary(q_random, q);
      if(ComputeNeighborhood(q_random))
      {
        q_children.push_back(q_random);
      }
    }
  }else{

    //############################################################################
    //(1) get a point q_next, which lies on the intersection of the line from q_parent to q with the neighborhood around q
    //############################################################################

    //q_last ---- q ---- q_next

    Configuration *q_next = new Configuration(Q1);
    double d_last_to_current = DistanceConfigurationConfiguration(q->parent_neighbor, q);
    double radius_current = q->GetRadius();
    Q1->getStateSpace()->interpolate(q->parent_neighbor->state, q->state, 1 + radius_current/d_last_to_current, q_next->state);
    if(ComputeNeighborhood(q_next)){
      q_children.push_back(q_next);
      ExpandSubsetNeighborhood(q, q_next, q_children);
    }

    // for(uint k = 0; k < NUMBER_OF_EXPANSION_SAMPLES-1; k++){
    //   Configuration *q_k = new Configuration(Q1);
    //   SampleNeighborhoodBoundaryHalfBall(q_k, q);
    //   if(ComputeNeighborhood(q_k)){
    //     q_children.push_back(q_k);
    //   }
    // }

    //if(ComputeNeighborhood(q_next)){
    //  q_children.push_back(q_next);
    //  ExpandSubsetNeighborhood(q, q_next, q_children);
    //}
  }

  for(uint k = 0; k < q_children.size(); k++){
    q_children.at(k)->parent_neighbor = q;
  }

  return q_children;
}
void QNG::ExpandSubsetNeighborhood(const Configuration *q_last, const Configuration *q_current, std::vector<Configuration*> &q_children)
{
  double radius_current = q_current->GetRadius();
  double radius_last = q_last->GetRadius();

  double radius_ratio = radius_current / radius_last;
  //radius ratio tells us about how the current radius has evolved from last
  //radius
  //Case1: radius_ratio > 1: 
  //   we encountered a bigger radius neighborhood. this is great, we should go
  //   into that direction
  //case2: radius_ratio < 1:
  //   linearly adjust sampling radius. the smaller our change, the bigger we
  //   should sample to escape the bad neighborhood
  double SAMPLING_RADIUS = 0.0;
  if(radius_ratio > 1){
    return;
  }else{
    //if(radius_ratio > 0.1){
    //  SAMPLING_RADIUS = (1.0-radius_ratio+0.1)*radius_current;
    //}else{
    //  SAMPLING_RADIUS = radius_current;
    //}
    SAMPLING_RADIUS = radius_current;
  }

  for(uint k = 0; k < NUMBER_OF_EXPANSION_SAMPLES-1; k++){
    Configuration *q_k = new Configuration(Q1);
    Q1_sampler->sampleUniformNear(q_k->state, q_current->state, SAMPLING_RADIUS);

    double d_last_to_k = DistanceQ1(q_last, q_k);

    //project onto ball of radius radius_last around q_last
    Q1->getStateSpace()->interpolate(q_last->state, q_k->state, min(radius_last/d_last_to_k,1.0), q_k->state);

    if(!ComputeNeighborhood(q_k)){
      continue;
    }
    q_children.push_back(q_k);
  }
}

QNG::Configuration* QNG::SampleCoverBoundary(){
  Configuration *q_random;
  if(!hasSolution){
    double r = rng_.uniform01();
    if(r<goalBias){
      q_random = BaseT::SampleCoverBoundary("goal");
    }else{
      q_random = BaseT::SampleCoverBoundary("voronoi");
    }
  }else{
    std::cout << "hasSolution sampler NYI"<< std::endl;
    exit(0);
  }
  return q_random;
}
void QNG::AddToFastTrackConditional(std::vector<Configuration*> q_children)
{
  if(q_children.size()<=0) return;

  int best_index = -1;
  double best_radius = 0;
  for(uint k = 0; k < q_children.size(); k++){
    Configuration *qk = q_children.at(k);
    double radius_k = qk->GetRadius();
    if(radius_k > best_radius)
    {
      best_index = k;
      best_radius = radius_k;
    }
  }
  if(best_index<0){
    OMPL_ERROR("Could not find best configuration");
    std::cout << "considered configurations:" << std::endl;
    for(uint k = 0; k < q_children.size(); k++){
      Configuration *qk = q_children.at(k);
      //double radius = qk->GetRadius();
      std::cout << qk << std::endl;
      //Q1->printState(qk->state);
    }
    exit(0);

  }

  Configuration *q_parent = q_children.at(0)->parent_neighbor;

  //###########################################################################
  //DEBUG
  //NOTE: we need to assert that all children share the same parent!
  for(uint k = 0; k < q_children.size(); k++){
    Configuration *q_parent_k = q_children.at(k)->parent_neighbor;
    if(q_parent != q_parent_k){
      std::cout << std::string(80, '#') << std::endl;
      std::cout << "children do not have same parent" << std::endl;
      std::cout << std::string(80, '#') << std::endl;
      std::cout << "number of children: " << q_children.size() << std::endl;
      std::cout << "parent of child " << k << " has state:" << std::endl;
      Q1->printState(q_parent_k->state);
      std::cout << "parent of child " << 0 << " has state:" << std::endl;
      Q1->printState(q_parent->state);
      exit(0);
    }
  }
  //###########################################################################

  //we add if the next best radius is at least 90 percent of the current
  if(best_radius >= q_parent->GetRadius())
  {
    fast_track_configurations.push_back(q_children.at(best_index));
    if(verbose>0) std::cout << "[FAST_TRACK] added vertex with radius " << q_children.at(best_index)->GetRadius() << std::endl;
    if(verbose>0) Q1->printState(q_children.at(best_index)->state);
  }
}

