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

void QNG::Grow(double t)
{
  ob::PlannerTerminationCondition ptc( ob::timedPlannerTerminationCondition(t) );
  //#########################################################################
  //Do not grow the cover if it is saturated, i.e. it cannot be expanded anymore
  // --- we have successfully computed Q1_{free}, the free space of Q1
  //#########################################################################
  if(saturated) return;

  //#########################################################################
  //Sample a configuration different from the current cover
  //#########################################################################
  Configuration *q_random = SampleValid(ptc);
  if(q_random == nullptr) return;

  AddConfigurationToCover(q_random);

  // //try to continue going in that direction
  // const uint max_extension_steps = 1000;
  // uint step = 0;
  // while(!ptc && step++ < max_extension_steps)
  // {
  //   //q_random->number_attempted_expansions++;
  //   //pdf_all_vertices.update(static_cast<PDF_Element*>(q_random->GetPDFElement()), q_random->GetImportance());

  //   Configuration *q_last = q_random->parent_neighbor;
  //   Configuration *q_next = EstimateBestNextState(q_last, q_random);

  //   if(q_next == nullptr) break;

  //   // //note that q_random might have been removed if the neighborhood is a
  //   // //proper subset of the neighborhood of q_next
  //   // if(q_random == nullptr){
  //   //   Connect(q_nearest, q_next);
  //   // }else{
  //   //   Connect(q_random, q_next);
  //   //   pdf_all_vertices.update(static_cast<PDF_Element*>(q_random->GetPDFElement()), q_random->GetImportance());
  //   // }
  //   q_next->parent_neighbor = q_random;
  //   q_random = q_next;
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

  double d_last_to_current = DistanceConfigurationConfiguration(q_last, q_current);
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
    if(!AddConfigurationToCover(q_next))
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
  if(!AddConfigurationToCover(q_best))
  {
    return nullptr;
  }
  //#########################################################################
  //return q_next
  //#########################################################################
  return q_best;
}
