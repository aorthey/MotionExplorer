#include "rrt_quotient_sufficiency.h"
#include "planner/validitychecker/validity_checker_ompl.h"

using namespace ompl::geometric;
using namespace ompl::base;
using namespace og;

RRTQuotientSufficiency::RRTQuotientSufficiency(const base::SpaceInformationPtr &si, Quotient *previous_) : 
  RRTQuotient(si, previous_)
{
  setName("RRTQuotientSufficiency");
}
ompl::PDF<RRTQuotient::Configuration*> RRTQuotientSufficiency::GetConfigurationPDF()
{
  PDF<Configuration*> pdf;
  std::vector<Configuration *> configurations;
  if(tStart_){
    tStart_->list(configurations);
  }
  for (auto &configuration : configurations)
  {
    if(!(configuration->parent == nullptr)){
      pdf.add(configuration, configuration->parent_edge_weight);
    }
  }
  configurations.clear();
  if(tGoal_){
    tGoal_->list(configurations);
  }
  for (auto &configuration : configurations)
  {
    if(!(configuration->parent == nullptr)){
      pdf.add(configuration, configuration->parent_edge_weight);
    }
  }

  if(pdf.empty()){
    std::cout << "cannot sample empty(?) graph" << std::endl;
    exit(0);
  }
  return pdf;
}
bool RRTQuotientSufficiency::SampleGraph(ob::State *q_random_graph)
{

  PDF<Configuration*> pdf = GetConfigurationPDF();

  Configuration *q = pdf.sample(rng_.uniform01());
  double t = rng_.uniform01();
  if(t<0.05){
    double tc = rng_.uniform01();
    const ob::State *from = connectionPoint_.first;
    const ob::State *to = connectionPoint_.second;
    M1->getStateSpace()->interpolate(from, to, tc, q_random_graph);
  }else{
    const ob::State *from = q->state;
    const ob::State *to = q->parent->state;
    M1->getStateSpace()->interpolate(from, to, t, q_random_graph);
  }

  return true;
}
