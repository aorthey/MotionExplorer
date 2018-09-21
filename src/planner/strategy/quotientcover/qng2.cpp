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
  NUMBER_OF_EXPANSION_SAMPLES = Q1->getStateDimension()+2;
  std::cout << "Number of expansion samples: " << NUMBER_OF_EXPANSION_SAMPLES << std::endl;
}

QNG2::~QNG2(void)
{
}

void QNG2::clear()
{
  BaseT::clear();
  firstRun = true;
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
  Connect(q_nearest, q_random);
}

void QNG2::Connect(Configuration *q_from, Configuration *q_next)
{
  //terminate condition (1) reached q_next
  const double d_from_to_next = DistanceNeighborhoodNeighborhood(q_from, q_next);
  if(d_from_to_next < 1e-10){
    if(ComputeNeighborhood(q_next)){
      q_next->parent_neighbor = q_from;
      AddConfigurationToCoverWithoutAddingEdges(q_next);
    }
    return;
  }

  std::vector<Configuration*> q_children = GenerateCandidateDirections(q_from, q_next);

  double radius_largest = 0;
  uint idx_largest = 0;
  for(uint k = 0; k < q_children.size(); k++)
  {
    Configuration *q_k = q_children.at(k);
    double r = q_k->GetRadius();
    if(r > radius_largest)
    {
      radius_largest = r;
      idx_largest = k;
    }
    q_k->parent_neighbor = q_from;
    AddConfigurationToCoverWithoutAddingEdges(q_k);
  }

  //terminate conditions
  //(1) next neighborhood too small
  if(radius_largest < 0.1*q_from->GetRadius()) return;

  //(3) no feasible neighborhoods
  if(!q_children.empty())
  {
    return Connect(q_children.at(idx_largest), q_next);
  }
}
std::vector<QuotientChartCover::Configuration*> QNG2::GenerateCandidateDirections(Configuration *q_from, Configuration *q_next)
{
  std::vector<Configuration*> q_children;

  //            \                                           |
  //             \q_k                                       |
  //              \                                         |
  // q_from        |q_proj                      q_next      |
  //              /                                         |
  //             /                                          |
  Configuration *q_proj = new Configuration(Q1);
  const double radius_from = q_from->GetRadius();
  double d = DistanceConfigurationConfiguration(q_from, q_next);
  Q1->getStateSpace()->interpolate(q_from->state, q_next->state, radius_from/d, q_proj->state);

  bool isProjectedFeasible = ComputeNeighborhood(q_proj);
  if(isProjectedFeasible)
  {
    q_proj->parent_neighbor = q_from;
    q_children.push_back(q_proj);
  }
  for(uint k = 0; k < NUMBER_OF_EXPANSION_SAMPLES; k++){
    Configuration *q_k = new Configuration(Q1);

    if(isProjectedFeasible){
      Q1_sampler->sampleUniformNear(q_k->state, q_proj->state, 0.5*q_from->GetRadius());
    }else{
      SampleNeighborhoodBoundaryHalfBall(q_k, q_from);
    }
    //SampleNeighborhoodBoundaryHalfBall(q_k, q_from);

    const double d_from_to_k = DistanceConfigurationConfiguration(q_from, q_k);
    Q1->getStateSpace()->interpolate(q_from->state, q_k->state, radius_from/d_from_to_k, q_k->state);

    if(ComputeNeighborhood(q_k))
    {
      q_k->parent_neighbor = q_from;
      q_children.push_back(q_k);
    }
  }
  return q_children;
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


//############################################################################
//Quotient Space Sampling Strategies
//############################################################################
QNG2::Configuration* QNG2::Sample()
{
  //first run should always go towards the goal (many environments can be solved
  //in that way, similar to potential field approach/generalized interpolation
  //towards goal --- it is the first thing
  //one should try).

  Configuration *q_random = new Configuration(Q1);
  if(firstRun){
    SampleGoal(q_random);
    firstRun = false;
    return q_random;
  }

  if(!hasSolution){
    double r = rng_.uniform01();
    if(r<goalBias){
      SampleGoal(q_random);
    }else{
      SampleUniform(q_random);
    }
  }else{
    SampleUniform(q_random);
    std::cout << "TODO: Phase2 Sampler only uses uniform."<< std::endl;
  }
  return q_random;
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
