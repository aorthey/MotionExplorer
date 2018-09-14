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
}

QNG::~QNG(void)
{
}

void QNG::setup(void)
{
  BaseT::setup();
  if(setup_)
  {
    //put the start configuration on the fast track
    //here is why: voronoi bias can seriously mislead the expansion of the first
    //node. what we want is to pick a random direction in cspace, this cannot be
    //guaranteed with voronoi bias
    fast_track_configurations.push_back(q_start);
  }
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

  if(fast_track_configurations.size()>0){
    //expand fast track neighborhoods first
    uint k = rng_.uniformInt(0, fast_track_configurations.size()-1);
    Configuration *q_fast_track = fast_track_configurations.at(k);

    std::cout << "fast track expansions of" << std::endl;
    Q1->printState(q_fast_track->state);

    std::vector<Configuration*> q_children = ExpandNeighborhood(q_fast_track);
    fast_track_configurations.erase(fast_track_configurations.begin()+k);
    //add the best child to the fast track if the children are all bigger or
    //equal to q
    AddToFastTrackConditional(q_children);

    for(uint k = 0; k < q_children.size(); k++){
      AddConfigurationToCover(q_children.at(k));
    }

  }else{
    double r = rng_.uniform01();
    if(importanceSamplingBias<r){
      Configuration *q_important = pdf_all_configurations.sample(rng_.uniform01());
      q_important->number_attempted_expansions++;
      pdf_all_configurations.update(static_cast<PDF_Element*>(q_important->GetPDFElement()), q_important->GetImportance());

      std::vector<Configuration*> q_children = ExpandNeighborhood(q_important);
      AddToFastTrackConditional(q_children);

      for(uint k = 0; k < q_children.size(); k++){
        AddConfigurationToCover(q_children.at(k));
      }
    }else{
      //classical RRT-like expansion with goal-bias
      Configuration *q_random = SampleCoverBoundaryValid(ptc);
      if(q_random == nullptr) return;
      AddConfigurationToCover(q_random);
    }
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
      }else{
        std::cout << "rejected" << std::endl;
        Q1->printState(q_random->state);
      }
    }
  }else{
    //(1) get a point q_next, which lies on the intersection of the line from q_parent to q with the neighborhood around q

    //q_parent ---- q ---- q_next
    Configuration *q_next = new Configuration(Q1);
    double d_last_to_current = DistanceConfigurationConfiguration(q->parent_neighbor, q);
    double radius_current = q->GetRadius();
    Q1->getStateSpace()->interpolate(q->parent_neighbor->state, q->state, 1 + radius_current/d_last_to_current, q_next->state);
    if(ComputeNeighborhood(q_next)){
      q_children.push_back(q_next);
    }
    //TODO sample around q_next

  }
  for(uint k = 0; k < q_children.size(); k++){
    q_children.at(k)->parent_neighbor = q;
  }

  return q_children;
}

void QNG::AddToFastTrackConditional(std::vector<Configuration*> q_vector)
{
  if(q_vector.size()<=0) return;

  int best_index = -1;
  double best_radius = 0;
  for(uint k = 0; k < q_vector.size(); k++){
    Configuration *qk = q_vector.at(k);
    double radius = qk->GetRadius();
    if(radius >= best_radius)
    {
      best_index = k;
      best_radius = radius;
    }
  }
  if(best_index<0){
    OMPL_ERROR("Could not find best configuration");
    std::cout << "considered configurations:" << std::endl;
    for(uint k = 0; k < q_vector.size(); k++){
      Configuration *qk = q_vector.at(k);
      //double radius = qk->GetRadius();
      std::cout << qk << std::endl;
      //Q1->printState(qk->state);
    }
    exit(0);

  }
  fast_track_configurations.push_back(q_vector.at(best_index));

}

// QNG::Configuration* QNG::EstimateBestNextState(Configuration *q_last, Configuration *q_current)
// {
//   uint K_samples = 3; //how many samples to test for best direction (depends maybe also on radius)

//   Configuration *q_next = new Configuration(Q1);

//   if(verbose>1){
//     std::cout << "from" << std::endl;
//     Q1->printState(q_last->state);
//     std::cout << "to" << std::endl;
//     Q1->printState(q_current->state);
//   }

//   double d_last_to_current = DistanceConfigurationConfiguration(q_last, q_current);
//   double radius_current = q_current->GetRadius();
//   Q1->getStateSpace()->interpolate(q_last->state, q_current->state, 1 + radius_current/d_last_to_current, q_next->state);

//   //#######################################################################
//   // We sample the distance function to obtain K samples {q_1,\cdots,q_K}.
//   // For each $q_k$ we compute the value of the distance function dk_radius.
//   // Then we employ two strategies to choose the next sample to follow:
//   //
//   //either follow isolines : min( fabs(radius_k_sample - current_radius) )
//   //or follow the steepest ascent: max(radius_k_sample)

//   double radius_best = DistanceInnerRobotToObstacle(q_next->state);
//   if(verbose>1){
//     std::cout << "next" << std::endl;
//     Q1->printState(q_next->state);
//     std::cout << "radius " << radius_best << std::endl;
//   }

//   //double radius_best = q_next->GetRadius();
//   double SAMPLING_RADIUS = 0.1*radius_current;
//   double radius_ratio = radius_best / radius_current;
//   if(radius_ratio > 1){
//     //we encountered a bigger radius neighborhood. this is great, we should go
//     //into that direction
//     if(!AddConfigurationToCover(q_next))
//     {
//       return nullptr;
//     }
//     return q_next;
//   }else{
//     //if next neighborhood is exceptionally small, try to sample more broadly.
//     //Otherwise sample smaller. This corresponds to having more speed in the
//     //momentum of sampling.
//     if(radius_ratio > 0.1){
//       SAMPLING_RADIUS = (1.0-radius_ratio+0.1)*radius_current;
//     }else{
//       SAMPLING_RADIUS = radius_current;
//     }
//   }

//   //#######################################################################
//   //keep q_next constant
//   q_next = const_cast<Configuration*>(q_next);
//   Configuration *q_best = q_next;

//   for(uint k = 0; k < K_samples; k++)
//   {
//     //obtain sample q_k, and radius radius_k
//     Configuration *q_k = new Configuration(Q1);
//     Q1_sampler->sampleUniformNear(q_k->state, q_next->state, SAMPLING_RADIUS);
//     double d_current_to_k = DistanceQ1(q_k, q_current);
//     Q1->getStateSpace()->interpolate(q_current->state, q_k->state, radius_current/d_current_to_k, q_k->state);

//     double radius_next = DistanceInnerRobotToObstacle(q_k->state);

//     if(verbose>1){
//       std::cout << "q_k" << std::endl;
//       Q1->printState(q_k->state);
//       std::cout << "radius: " << radius_next << std::endl;
//     }

//     if(q_best->isSufficientFeasible && !q_k->isSufficientFeasible)
//     {
//       q_best = q_k;
//       radius_best = radius_next;
//     }else{
//       if(radius_next > radius_best)
//       {
//         q_best = q_k;
//         radius_best = radius_next;
//       }else{
//         q_k->Remove(Q1);
//         continue;
//       }
//     }
//   }
//   if(!AddConfigurationToCover(q_best))
//   {
//     return nullptr;
//   }
//   //#########################################################################
//   //return q_next
//   //#########################################################################
//   return q_best;
// }

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
