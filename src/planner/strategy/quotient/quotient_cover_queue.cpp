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

QuotientCoverQueue::QuotientCoverQueue(const base::SpaceInformationPtr &si, Quotient *parent ): BaseT(si, parent)
{
  if(parent != nullptr) verbose = 1;
}
// ">" operator: smallest value is top in queue
// "<" operator: largest value is top in queue (default)
bool QuotientCoverQueue::CmpCandidateConfigurationPtrs::operator()(const Configuration* lhs, const Configuration* rhs) const
{
   return lhs->GetRadius() < rhs->GetRadius();
}
// ">" operator: smallest value is top in queue
// "<" operator: largest value is top in queue (default)
bool QuotientCoverQueue::CmpMemberConfigurationPtrs::operator()(const Configuration* lhs, const Configuration* rhs) const
{
  uint klhs = max(lhs->number_attempted_expansions,1U);
  uint krhs = max(rhs->number_attempted_expansions,1U);
  if(krhs < klhs){
    return true;
  }else{
    if(krhs > klhs){
      return false;
    }else{
      return lhs->GetRadius() < rhs->GetRadius();
    }
  }
}

QuotientCoverQueue::~QuotientCoverQueue(void)
{
}

void QuotientCoverQueue::setup()
{
  BaseT::setup();
  firstRun = true;
}
void QuotientCoverQueue::AddConfigurationToPriorityQueue(Configuration *q)
{
  if(q->parent_neighbor == nullptr && !q->isStart){
    std::cout << "Tried adding configuration without parent" << std::endl;
    QuotientCover::Print(q, false);
    exit(0);
  }
  if(q->GetRadius() > minimum_neighborhood_radius){
    priority_queue_candidate_configurations.push(q);
  }
}

void QuotientCoverQueue::clear()
{
  BaseT::clear();
  while(!priority_queue_candidate_configurations.empty()) 
  {
    priority_queue_candidate_configurations.pop();
  }
  NUMBER_OF_EXPANSION_SAMPLES = (Q1->getStateDimension()+1)*1;
  firstRun = true;
}

QuotientCover::Vertex QuotientCoverQueue::AddConfigurationToCover(Configuration *q)
{
  Vertex v = BaseT::AddConfigurationToCover(q);
  priority_queue_member_configurations.push(q);
  return v;
}

void QuotientCoverQueue::PrintQueue(int n_head)
{
  std::cout << std::string(80, '-') << std::endl;
  std::cout << "PriorityQueue" << std::endl;
  std::cout << std::string(80, '-') << std::endl;

  //Make temporary copy, then iterate over it
  CandidateConfigurationPriorityQueue printout_queue = priority_queue_candidate_configurations;

  int n_ctr = 0;
  while(!printout_queue.empty() && (n_ctr++ <= n_head)){

    Configuration *q_member = printout_queue.top();
    std::cout << q_member->index << " : " << q_member->number_attempted_expansions << "|" << q_member->GetRadius() << std::endl;
    printout_queue.pop();
  }
}

QuotientCoverQueue::Configuration* QuotientCoverQueue::SampleCoverBoundary(){
  Configuration *q_random;
  if(!hasSolution){
    //phase1 sampling: solution has not been found
    q_random = BaseT::SampleCoverBoundary("voronoi");
  }else{
    //phase2 sampling: grow solution and associated roadmap
    std::cout << "Phase2 (hasSolution) sampler NYI"<< std::endl;
    exit(0);
  }
  return q_random;
}

void QuotientCoverQueue::AddConfigurationToPDF(Configuration *q)
{
  PDF_Element *q_element = pdf_all_configurations.add(q, q->GetImportance());
  q->SetPDFElement(q_element);

  if(!q->isSufficientFeasible){
    //all necessary elements are equal
    PDF_Element *q_necessary_element = pdf_necessary_configurations.add(q, 1);
    q->SetNecessaryPDFElement(q_necessary_element);
  }
}

const QuotientCoverQueue::CandidateConfigurationPriorityQueue& QuotientCoverQueue::GetPriorityQueue()
{
  return priority_queue_candidate_configurations;
}

void QuotientCoverQueue::Print(std::ostream& out) const
{
  BaseT::Print(out);
  out << std::endl << " |------ [Queue] has " << priority_queue_candidate_configurations.size() << " configurations left in priority queue.";
}

QuotientCoverQueue::Configuration* QuotientCoverQueue::SampleUniformQuotientCover(ob::State *state) 
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
