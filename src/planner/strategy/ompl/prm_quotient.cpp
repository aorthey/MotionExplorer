#include "prm_quotient.h"
#include "planner/cspace/cspace.h"
#include "planner/validitychecker/validity_checker_ompl.h"

#include <ompl/datastructures/PDF.h>
#include <boost/foreach.hpp>
#include <boost/graph/graphviz.hpp>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/spaces/SO3StateSpace.h>

using namespace og;
using namespace ob;
#define foreach BOOST_FOREACH

namespace ompl
{
  namespace magic
  {
    static const unsigned int MAX_RANDOM_BOUNCE_STEPS = 5;
    static const double ROADMAP_BUILD_TIME = 0.01;
    static const unsigned int DEFAULT_NEAREST_NEIGHBORS = 10;
  }
}

PRMQuotient::PRMQuotient(const ob::SpaceInformationPtr &si, Quotient *previous_ ):
  og::PRMBasic(si, previous_)
{
  setName("PRMQuotient"+std::to_string(id));
}

PRMQuotient::~PRMQuotient()
{
}

void PRMQuotient::clear()
{
  PRMBasic::clear();
  isSampled = false; 
}


void PRMQuotient::Init()
{
  checkValidity();
  auto *goal = dynamic_cast<ob::GoalSampleableRegion *>(pdef_->getGoal().get());

  if (goal == nullptr){
    OMPL_ERROR("%s: Unknown type of goal", getName().c_str());
    exit(0);
  }

  while (const ob::State *st = pis_.nextStart()){
    startM_.push_back(addMilestone(si_->cloneState(st)));
  }
  if (startM_.empty()){
    OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
    exit(0);
  }
  if (!goal->couldSample()){
    OMPL_ERROR("%s: Insufficient states in sampleable goal region", getName().c_str());
    exit(0);
  }

  if (goal->maxSampleCount() > goalM_.size() || goalM_.empty()){
    const ob::State *st = pis_.nextGoal();
    if (st != nullptr){
      goalM_.push_back(addMilestone(si_->cloneState(st)));
    }
  }
  unsigned long int nrStartStates = boost::num_vertices(g_);
  OMPL_INFORM("%s: Starting planning with %lu states already in datastructure", getName().c_str(), nrStartStates);

  og::PRMQuotient *PRMprevious = static_cast<og::PRMQuotient*>(previous);

  if(PRMprevious!=nullptr){
    for(uint k = 0; k < startM_.size(); k++){
      associatedVertexSourceProperty_[startM_.at(k)] = PRMprevious->startM_.at(k);
      associatedVertexTargetProperty_[startM_.at(k)] = PRMprevious->startM_.at(k);
    }
    for(uint k = 0; k < goalM_.size(); k++){
      associatedVertexSourceProperty_[goalM_.at(k)] = PRMprevious->goalM_.at(k);
      associatedVertexTargetProperty_[goalM_.at(k)] = PRMprevious->goalM_.at(k);
    }
  }
}



og::PRMBasic::Vertex PRMQuotient::addMilestone(base::State *state)
{
  Vertex m = PRMBasic::addMilestone(state);
    
  og::PRMQuotient *PRMprevious = static_cast<og::PRMQuotient*>(previous);
  if(PRMprevious != nullptr && PRMprevious->isSampled){
    //this is not always correct!
    associatedVertexSourceProperty_[m] = PRMprevious->lastSourceVertexSampled;
    associatedVertexTargetProperty_[m] = PRMprevious->lastTargetVertexSampled;
    associatedTProperty_[m] = PRMprevious->lastTSampled;
  }
  return m;
}

void PRMQuotient::setup()
{
  og::PRMBasic::setup();
  nn_->setDistanceFunction([this](const Vertex a, const Vertex b)
                           {
                             return Distance(a,b);
                           });
}


void PRMQuotient::Grow(double t){
  double T_grow = (2.0/3.0)*t;
  double T_expand = (1.0/3.0)*t;
  growRoadmap(ob::timedPlannerTerminationCondition(T_grow), xstates[0]);
  expandRoadmap( ob::timedPlannerTerminationCondition(T_expand), xstates);
}
double PRMQuotient::GetSamplingDensity()
{
  if(previous == nullptr){
    return (double)num_vertices(g_)/(double)M1->getSpaceMeasure();
    //return (double)num_vertices(g_);
  }else{
    //get graph length
    //double Lprev = 0.0;
    //og::PRMQuotient *PRMprevious = static_cast<og::PRMQuotient*>(previous);
    //const Graph gprev = PRMprevious->getRoadmap();
    //foreach (Edge e, boost::edges(gprev))
    //{
    //  EdgeProperty ep = get(boost::edge_weight_t(), gprev, e);
    //  ob::Cost weight = ep.getCost();
    //  Lprev += weight.value();
    //}
    return (double)num_vertices(g_)/(M1->getSpaceMeasure());
    //return (double)num_vertices(g_);
  }
}


void PRMQuotient::getPlannerData(base::PlannerData &data) const
{
  PRMBasic::getPlannerData(data);
}

bool PRMQuotient::Sample(ob::State *q_random)
{
  if(previous == nullptr){
    //without any underlying CSpace, the behavior is equal to PRM
    return sampler_->sample(q_random);
  }else{
    //Adjusted sampling function: Sampling in G0 x C1

    ob::SpaceInformationPtr M0 = previous->getSpaceInformation();
    base::State *s_C1 = C1->allocState();
    base::State *s_M0 = M0->allocState();

    C1_sampler->sampleUniform(s_C1);
    previous->SampleGraph(s_M0);
    mergeStates(s_M0, s_C1, q_random);

    C1->freeState(s_C1);
    M0->freeState(s_M0);

    return M1->isValid(q_random);
  }
}

bool PRMQuotient::SampleGraph(ob::State *q_random_graph)
{
  PDF<Edge> pdf = GetEdgePDF();
  if(pdf.empty()){
    std::cout << "cannot sample empty(?) graph" << std::endl;
    exit(0);
  }

  auto checkerPtr = static_pointer_cast<OMPLValidityCheckerNecessarySufficient>(M1->getStateValidityChecker());
  bool foundNecessary = false;
  while(!foundNecessary)
  {
    Edge e = pdf.sample(rng_.uniform01());
    double t = rng_.uniform01();
    const Vertex v1 = boost::source(e, g_);
    const Vertex v2 = boost::target(e, g_);
    const ob::State *from = stateProperty_[v1];
    const ob::State *to = stateProperty_[v2];
    M1->getStateSpace()->interpolate(from, to, t, q_random_graph);

    lastSourceVertexSampled = v1;
    lastTargetVertexSampled = v2;
    lastTSampled = t;
    if(!checkerPtr->IsSufficient(q_random_graph)){
      foundNecessary = true;
    }
  }

  isSampled = true;

  return true;

}

ompl::PDF<og::PRMBasic::Edge> PRMQuotient::GetEdgePDF()
{
  PDF<Edge> pdf;
  foreach (Edge e, boost::edges(g_))
  {
    const Vertex v1 = boost::source(e, g_);
    if(sameComponent(v1, startM_.at(0))){
      ob::Cost weight = get(boost::edge_weight_t(), g_, e).getCost();
      pdf.add(e, weight.value());
    }
  }
  return pdf;
}

double PRMQuotient::Distance(const Vertex a, const Vertex b) const
{
  return si_->distance(stateProperty_[a], stateProperty_[b]);
}

bool PRMQuotient::Connect(const Vertex a, const Vertex b)
{
  return PRMBasic::Connect(a,b);
}
