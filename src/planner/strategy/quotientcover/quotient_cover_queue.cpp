#include "common.h"
#include "quotient_cover_queue.h"
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

QuotientChartCoverQueue::QuotientChartCoverQueue(const base::SpaceInformationPtr &si, Quotient *parent ): BaseT(si, parent)
{
  if(parent != nullptr) verbose = 1;
}

QuotientChartCoverQueue::~QuotientChartCoverQueue(void)
{
}

void QuotientChartCoverQueue::setup()
{
  BaseT::setup();
  NUMBER_OF_EXPANSION_SAMPLES = (Q1->getStateDimension()+1)*1;
  firstRun = true;
}
void QuotientChartCoverQueue::clear()
{
  BaseT::clear();
  while(!priority_configurations.empty()) 
  {
    Configuration *q = priority_configurations.top();
    if(q!=nullptr) q->Remove(Q1);
    priority_configurations.pop();
  }
}

void QuotientChartCoverQueue::Grow(double t)
{
  if(saturated) return;

  ob::PlannerTerminationCondition ptc( ob::timedPlannerTerminationCondition(t) );

  //############################################################################
  //Get the configuration with the largest neighborhood (NBH) from the queue
  //############################################################################
  Configuration *q = nullptr;
  if(firstRun)
  {
    q = q_start;
    firstRun = false;
  }else{
    if(priority_configurations.empty()){
      //try random directions
      //saturated = true;
      if(verbose>0) std::cout << "Space got saturated." << std::endl;
      Configuration *q_random = SampleCoverBoundaryValid(ptc);
      if(q_random == nullptr) return;
      priority_configurations.push(q_random);
    }
    q = priority_configurations.top();
    priority_configurations.pop();
    if(IsConfigurationInsideCover(q)){
      q->Remove(Q1);
      q=nullptr;
      return;
    }
    AddConfigurationToCoverWithoutAddingEdges(q);
  }

  //############################################################################
  // Expand largest NBH. This expansion can be biased towards the goal/random
  // directions/voronoi regions, etcetera. The actual algorithm will implement
  // this function.
  //############################################################################

  std::vector<Configuration*> q_children = ExpandNeighborhood(q, NUMBER_OF_EXPANSION_SAMPLES);
  for(uint k = 0; k < q_children.size(); k++)
  {
    priority_configurations.push(q_children.at(k));
  }

  //add different biases to remove planner from getting stuck
  Configuration *q_random = SampleCoverBoundaryValid(ptc);
  if(q_random == nullptr) return;
  priority_configurations.push(q_random);

}

QuotientChartCoverQueue::Configuration* QuotientChartCoverQueue::SampleCoverBoundary(){
  Configuration *q_random;
  if(!hasSolution){
    //phase1 sampling: solution has not been found
    double r = rng_.uniform01();
    if(r<goalBias){
      q_random = BaseT::SampleCoverBoundary("goal");
    }else{
      q_random = BaseT::SampleCoverBoundary("voronoi");
    }
  }else{
    //phase2 sampling: grow solution and associated roadmap
    std::cout << "Phase2 (hasSolution) sampler NYI"<< std::endl;
    exit(0);
  }
  return q_random;
}

void QuotientChartCoverQueue::AddConfigurationToPDF(Configuration *q)
{
  PDF_Element *q_element = pdf_all_configurations.add(q, q->GetImportance());
  q->SetPDFElement(q_element);

  if(!q->isSufficientFeasible){
    //all necessary elements are equal
    PDF_Element *q_necessary_element = pdf_necessary_configurations.add(q, 1);
    q->SetNecessaryPDFElement(q_necessary_element);
  }
}
void QuotientChartCoverQueue::Print(std::ostream& out) const
{
  BaseT::Print(out);
  out << std::endl << " |------ [Queue] has " << priority_configurations.size() << " configurations left in priority queue.";
}
