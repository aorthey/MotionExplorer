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
// ">" operator: smallest value is top in queue
// "<" operator: largest value is top in queue (default)
bool QuotientCoverQueue::CmpGoalDistancePtrs::operator()(const Configuration* lhs, const Configuration* rhs) const
{
   return lhs->GetGoalDistance() > rhs->GetGoalDistance();
}

QuotientCover::Configuration* QuotientCoverQueue::PriorityQueueNearestToGoal_Top()
{
  if(configurations_sorted_by_nearest_to_goal.empty()){
    std::cout << "QUEUE IS EMPTY" << std::endl;
  }
  Configuration *q_nearest = configurations_sorted_by_nearest_to_goal.top();
  while(q_nearest->index < 0 && !configurations_sorted_by_nearest_to_goal.empty()){
    configurations_sorted_by_nearest_to_goal.pop();
    q_nearest = configurations_sorted_by_nearest_to_goal.top();
  }
  return q_nearest;
}

bool QuotientCoverQueue::PriorityQueueCandidate_IsEmpty()
{
  return configurations_sorted_by_nearest_to_goal.empty();
}
QuotientCover::Configuration* QuotientCoverQueue::PriorityQueueCandidate_PopTop()
{
  Configuration *q = nullptr;
  while(q==nullptr && !priority_queue_candidate_configurations.empty()){
    q = priority_queue_candidate_configurations.top();
    priority_queue_candidate_configurations.pop();
  }
  return q;
}

QuotientCoverQueue::~QuotientCoverQueue(void)
{
}

void QuotientCoverQueue::setup()
{
  BaseT::setup();
}
void QuotientCoverQueue::AddConfigurationToPriorityQueue(Configuration *q)
{
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
  while(!configurations_sorted_by_nearest_to_goal.empty()) 
  {
    configurations_sorted_by_nearest_to_goal.pop();
  }
  pdf_configurations_connectivity.clear();
  nearest_to_goal_has_changed = true;
}

void QuotientCoverQueue::AddEdge(Configuration *q_from, Configuration *q_to)
{
  BaseT::AddEdge(q_from, q_to);
  PDFConnectivityUpdate(q_from);
  PDFConnectivityUpdate(q_to);
}

QuotientCover::Vertex QuotientCoverQueue::AddConfigurationToCoverGraph(Configuration *q)
{
  Vertex v = BaseT::AddConfigurationToCoverGraph(q);
  priority_queue_member_configurations.push(q);

  PDFConnectivityAdd(q);

  configurations_sorted_by_nearest_to_goal.push(q);
  if(q == configurations_sorted_by_nearest_to_goal.top()){
    nearest_to_goal_has_changed = true;
  }

  return v;
}
void QuotientCoverQueue::RemoveConfigurationFromCover(Configuration *q)
{
  PDFConnectivityRemove(q);
  BaseT::RemoveConfigurationFromCover(q);
}

bool QuotientCoverQueue::NearestToGoalHasChanged()
{
  if(nearest_to_goal_has_changed){
    nearest_to_goal_has_changed = false;
    return true;
  }else{
    return false;
  }
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

void QuotientCoverQueue::Print(std::ostream& out) const
{
  BaseT::Print(out);
  out << std::endl << " |------ [Queue] has " << priority_queue_candidate_configurations.size() << " configurations left in priority queue.";
}

void QuotientCoverQueue::PDFConnectivityUpdate(Configuration *q)
{
  if(q->GetConnectivityPDFElement()==nullptr){
    std::cout << std::string(80, '#') << std::endl;
    std::cout << "Trying to update the PDF element for connectivity." << std::endl;
    std::cout << "But does not exist. Configuration:" << std::endl;
    Print(q, false);
    std::cout << "Value Connectivity: " << ValueConnectivity(q) << std::endl; 
    std::cout << std::string(80, '#') << std::endl;
    exit(0);
  }
  pdf_configurations_connectivity.update(static_cast<PDF_Element*>(q->GetConnectivityPDFElement()), ValueConnectivity(q));
}
void QuotientCoverQueue::PDFConnectivityAdd(Configuration *q)
{
  PDF_Element *q_element = pdf_configurations_connectivity.add(q, ValueConnectivity(q));
  q->SetConnectivityPDFElement(q_element);
}
void QuotientCoverQueue::PDFConnectivityRemove(Configuration *q)
{
  pdf_configurations_connectivity.remove(static_cast<PDF_Element*>(q->GetConnectivityPDFElement()));
}

double QuotientCoverQueue::ValueConnectivity(Configuration *q)
{
  //Vertex v = get(indexToVertex, q->index);
  //QuotientCover::Print(q, false);
  double d_alpha = q->number_attempted_expansions;
  //double d_alpha = boost::degree(v, graph)+1;
  //double d_connectivity = q->GetRadius()/d_alpha;
  double d_connectivity = 1.0/(d_alpha+1);
  return d_connectivity;
}
QuotientCover::Configuration* QuotientCoverQueue::GetConfigurationLowConnectivity()
{
  return pdf_configurations_connectivity.sample(rng_.uniform01());
}
