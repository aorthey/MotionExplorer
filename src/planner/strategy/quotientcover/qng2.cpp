#include "common.h"
#include "qng2.h"
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

QNG2::QNG2(const base::SpaceInformationPtr &si, Quotient *parent ): BaseT(si, parent)
{
  setName("QNG2"+std::to_string(id));
  NUMBER_OF_EXPANSION_SAMPLES = Q1->getStateDimension()+1;
  if(parent!=nullptr) verbose=1;
}

QNG2::~QNG2(void)
{
}

void QNG2::clear()
{
  BaseT::clear();
}

void QNG2::setup(void)
{
  BaseT::setup();
}

void QNG2::Grow(double t)
{
  if(saturated) return;

  Configuration *q_random = Sample();
  Configuration *q_nearest = Nearest(q_random);

  //############################################################################
  //project random onto neighborhood of q_nearest
  //############################################################################
  const double radius_nearest = q_nearest->GetRadius();
  double d = DistanceQ1(q_nearest, q_random);
  Q1->getStateSpace()->interpolate(q_nearest->state, q_random->state, radius_nearest/d, q_random->state);

  ConnectRecurseLargest(q_nearest, q_random);
}

void QNG2::ConnectRecurseLargest(Configuration *q_from, Configuration *q_next)
{
  //############################################################################
  //(1) analyse next configuration
  //############################################################################
  const double radius_from = q_from->GetRadius();
  const double radius_next = q_next->GetRadius();

  if(ComputeNeighborhood(q_next))
  {
    q_next->parent_neighbor = q_from;
    AddConfigurationToCoverWithoutAddingEdges(q_next);
    const double radius_ratio = radius_next / radius_from;

    if(radius_ratio > 1){
      //############################################################################
      //(1a) next configuration is larger => terminate and go into that direction
      //############################################################################
      const double d_from_to_next = DistanceConfigurationConfiguration(q_from, q_next);
      Configuration *q_extend = new Configuration(Q1);
      Q1->getStateSpace()->interpolate(q_from->state, q_next->state, 1 + radius_next/d_from_to_next, q_extend->state);
      q_extend->parent_neighbor = q_next;
      return ConnectRecurseLargest(q_next, q_extend);
    }else{
      //############################################################################
      //(1b) next configuration is smaller or equal => spawn M configurations,
      //and go into the largest direction from there
      //############################################################################
      double largest_radius = 0;
      uint largest_idx = 0;
      std::vector<Configuration*> q_spawnlings;
      for(uint k = 0; k < NUMBER_OF_EXPANSION_SAMPLES-1; k++)
      {
        Configuration *q_k = new Configuration(Q1);
        Q1_sampler->sampleUniformNear(q_k->state, q_next->state, 0.5*q_next->GetRadius());

        const double d_next_to_k = DistanceQ1(q_next, q_k);
        Q1->getStateSpace()->interpolate(q_next->state, q_k->state, radius_next/d_next_to_k, q_k->state);

        if(ComputeNeighborhood(q_k))
        {
          q_k->parent_neighbor = q_next;
          //AddConfigurationToCoverWithoutAddingEdges(q_k);
          q_spawnlings.push_back(q_k);
          if(largest_radius < q_k->GetRadius()){
            largest_radius = q_k->GetRadius();
            largest_idx = q_spawnlings.size()-1;
          }
        }
      }
      for(uint k = 0; k < q_spawnlings.size(); k++){
        if(k!=largest_idx){
          Configuration *q = q_spawnlings.at(k);
          AddConfigurationToCoverWithoutAddingEdges(q);
        }
      }
      if(!q_spawnlings.empty())
      {
        //############################################################################
        //(1b1) expand into direction of configuration with largest neighborhood
        //############################################################################
        Configuration *q = q_spawnlings.at(largest_idx);
        std::cout << "largest directioN: " << q->GetRadius() << " index " << q->index << std::endl;
        return ConnectRecurseLargest(q_next, q_spawnlings.at(largest_idx));
      }else{
        //############################################################################
        //(1b2) all children are infeasible => terminate
        //############################################################################
        return;
      }
    }

  }else{
    //############################################################################
    //(1c) next configuration is infeasible => terminate
    //############################################################################
    return;
  }



}

//############################################################################
//Quotient Space Sampling Strategies
//############################################################################
QNG2::Configuration* QNG2::Sample()
{
  Configuration *q_random = new Configuration(Q1);
  if(!hasSolution){
    double r = rng_.uniform01();
    if(r<goalBias){
      SampleGoal(q_random);
    }else{
      SampleUniform(q_random);
    }
  }else{
    std::cout << "hasSolution sampler NYI"<< std::endl;
    exit(0);
  }
  return q_random;
}


void QNG2::AddConfigurationToPDF(Configuration *q)
{
  PDF_Element *q_element = pdf_all_configurations.add(q, q->GetImportance());
  q->SetPDFElement(q_element);
  if(!q->isSufficientFeasible){
    PDF_Element *q_necessary_element = pdf_necessary_configurations.add(q, 1);
    q->SetNecessaryPDFElement(q_necessary_element);
  }
}
QNG2::Configuration* QNG2::SampleQuotientCover(ob::State *state) 
{
  double r = rng_.uniform01();
  Configuration *q_coset = nullptr;
  if(r<shortestPathBias)
  {
    if(shortest_path_start_goal_necessary_vertices.empty())
    {
      std::cout << "[WARNING] shortest path does not have any necessary vertices! -- should be detected on CS" << std::endl;
      exit(0);
    }
    int k = rng_.uniformInt(0, shortest_path_start_goal_necessary_vertices.size()-1);
    const Vertex vk = shortest_path_start_goal_necessary_vertices.at(k);
    q_coset = graph[vk];
    Q1_sampler->sampleUniformNear(state, q_coset->state, q_coset->GetRadius());
  }else{
    q_coset = pdf_necessary_configurations.sample(rng_.uniform01());
    Q1_sampler->sampleUniformNear(state, q_coset->state, q_coset->GetRadius());
  }
  return q_coset;
}
