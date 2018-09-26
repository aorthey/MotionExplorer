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
  setName(typeid(*this).name()+std::to_string(id));
}

QNG::~QNG(void)
{
}

std::vector<QNG::Configuration*> QNG::ExpandNeighborhood(Configuration *q_current, const int M_samples)
{
  std::vector<Configuration*> q_children;
  if(q_current->parent_neighbor == nullptr)
  {
    //if no parent neighbor is available, just sample on the neighborhood boundary
    for(int k = 0; k < M_samples; k++)
    {
      Configuration *q_random = new Configuration(Q1);
      SampleNeighborhoodBoundary(q_random, q_current);
      if(ComputeNeighborhood(q_random))
      {
        q_children.push_back(q_random);
      }
    }
  }else{

    //############################################################################
    // Get projected sample
    //############################################################################
    Configuration *q_proj = new Configuration(Q1);
    Configuration *q_last = q_current->parent_neighbor;
    const double radius_current = q_current->GetRadius();
    const double radius_last = q_last->GetRadius();
    const double step_size = (radius_last+radius_current)/radius_last;
    Q1->getStateSpace()->interpolate(q_last->state, q_current->state, step_size, q_proj->state);

    //############################################################################
    // (1) q_proj is feasible: 
    //    (1a) neighborhood is bigger than current neighborhood -> return q_proj
    //    (1b) otherwise compute other neighborhoods
    // (2) q_proj is infeasible:
    //    (2a) sample broadly
    //############################################################################
    if(ComputeNeighborhood(q_proj))
    {
      q_children.push_back(q_proj);

      const double radius_proj = q_proj->GetRadius();
      const double radius_ratio = radius_proj / radius_current;
      if(radius_ratio < 1)
      {
        for(int k = 0; k < M_samples-1; k++)
        {
          Configuration *q_k = new Configuration(Q1);
          Q1_sampler->sampleUniformNear(q_k->state, q_proj->state, 0.5*radius_current);

          if(ComputeNeighborhood(q_k))
          {
            q_children.push_back(q_k);
          }
        }
      }

    }else{
      for(int k = 0; k < M_samples; k++)
      {
        Configuration *q_k = new Configuration(Q1);
        SampleNeighborhoodBoundaryHalfBall(q_k, q_current);

        if(ComputeNeighborhood(q_k))
        {
          q_children.push_back(q_k);
          //if(q_k->GetRadius() > 0.1*q_current->GetRadius()){
          //}
        }
      }
    }
  }

  for(uint k = 0; k < q_children.size(); k++){
    q_children.at(k)->parent_neighbor = q_current;
  }

  return q_children;
}
//void QNG::ExpandSubsetNeighborhood(const Configuration *q_last, const Configuration *q_current, std::vector<Configuration*> &q_children, const int M_samples)
//{
//  const double radius_current = q_current->GetRadius();
//  const double radius_last = q_last->GetRadius();
//  const double radius_ratio = radius_current / radius_last;
//  //radius ratio tells us about how the current radius has evolved from last
//  //radius
//  //Case1: radius_ratio > 1: 
//  //   we encountered a bigger radius neighborhood. this is great, we should go
//  //   into that direction
//  //case2: radius_ratio < 1:
//  //   linearly adjust sampling radius. the smaller our change, the bigger we
//  //   should sample to escape the bad neighborhood
//  double SAMPLING_RADIUS = 0.0;
//  if(radius_ratio > 1)
//  {
//    return;
//  }else{
//    //if(radius_ratio > 0.1){
//    //  SAMPLING_RADIUS = (1.0-radius_ratio+0.1)*radius_current;
//    //}else{
//    //  SAMPLING_RADIUS = radius_current;
//    //}
//    SAMPLING_RADIUS = radius_current;
//  }
//
//  for(int k = 0; k < M_samples; k++)
//  {
//    Configuration *q_k = new Configuration(Q1);
//    Q1_sampler->sampleUniformNear(q_k->state, q_current->state, SAMPLING_RADIUS);
//
//    const double d_last_to_k = DistanceQ1(q_last, q_k);
//
//    //project onto ball of radius radius_last around q_last
//    //Q1->getStateSpace()->interpolate(q_last->state, q_k->state, min(radius_last/d_last_to_k,1.0), q_k->state);
//    Q1->getStateSpace()->interpolate(q_last->state, q_k->state, radius_last/d_last_to_k, q_k->state);
//
//    if(ComputeNeighborhood(q_k))
//    {
//      q_children.push_back(q_k);
//    }
//  }
//}
//
//QNG::Configuration* QNG::SampleCoverBoundary(){
//  Configuration *q_random;
//  if(!hasSolution){
//    //phase1 sampling: solution has not been found
//    double r = rng_.uniform01();
//    if(r<goalBias){
//      q_random = BaseT::SampleCoverBoundary("goal");
//    }else{
//      q_random = BaseT::SampleCoverBoundary("voronoi");
//    }
//  }else{
//    //phase2 sampling: grow solution and associated roadmap
//    std::cout << "Phase2 (hasSolution) sampler NYI"<< std::endl;
//    exit(0);
//  }
//  return q_random;
//}
//
//void QNG::AddConfigurationToPDF(Configuration *q)
//{
//  PDF_Element *q_element = pdf_all_configurations.add(q, q->GetImportance());
//  q->SetPDFElement(q_element);
//
//  if(!q->isSufficientFeasible){
//    //all necessary elements are equal
//    PDF_Element *q_necessary_element = pdf_necessary_configurations.add(q, 1);
//    q->SetNecessaryPDFElement(q_necessary_element);
//  }
//}
//QNG::Configuration* QNG::SampleUniformQuotientCover(ob::State *state) 
//{
//  double r = rng_.uniform01();
//  Configuration *q_coset = nullptr;
//  if(r<shortestPathBias)
//  {
//    if(shortest_path_start_goal_necessary_vertices.empty())
//    {
//      std::cout << "[WARNING] shortest path does not have any necessary vertices! -- should be detected on CS" << std::endl;
//      exit(0);
//    }
//    int k = rng_.uniformInt(0, shortest_path_start_goal_necessary_vertices.size()-1);
//    const Vertex vk = shortest_path_start_goal_necessary_vertices.at(k);
//    q_coset = graph[vk];
//    Q1_sampler->sampleUniformNear(state, q_coset->state, q_coset->GetRadius());
//  }else{
//    q_coset = pdf_necessary_configurations.sample(rng_.uniform01());
//    Q1_sampler->sampleUniformNear(state, q_coset->state, q_coset->GetRadius());
//  }
//  return q_coset;
//}
