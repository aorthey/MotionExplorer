#include "prm_quotient.h"
#include "planner/cspace/cspace.h"

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
//  const StateSpacePtr M1_space = M1->getStateSpace();
//  M0_subspaces = 0;
//  M1_subspaces = 0;
//
//  id = counter++;
//  std::cout << "--- Level " << id << " QuotientSpace" << std::endl;
//  setName("PRMQuotient"+to_string(id));
//  if(previous == nullptr){
//    std::cout << "M" << id <<" (dimension: " << M1_space->getDimension() << ") contains " << M1_subspaces << " subspaces of type " << M1_space->getName() << " " << M1_space->getType() << std::endl;
//  }else{
//    const StateSpacePtr M0_space = previous->getSpaceInformation()->getStateSpace();
//
//    //create standalone CSpace M1/M0
//    if(!M1_space->isCompound()){
//      std::cout << "Cannot Split a non-compound configuration space" << std::endl;
//      exit(0);
//    }else{
//
//      //get count of subspaces in M0
//      if(!M0_space->isCompound()){
//        M0_subspaces=1;
//      }else{
//        M0_subspaces = M0_space->as<ob::CompoundStateSpace>()->getSubspaceCount();
//      }
//
//      //get count of subspaces in M1
//      if(!M1_space->isCompound()){
//        std::cout << "M1 needs to be compound state" << std::endl;
//        exit(0);
//      }
//
//      ob::CompoundStateSpace *M1_compound = M1_space->as<ob::CompoundStateSpace>();
//      const std::vector<StateSpacePtr> M1_decomposed = M1_compound->getSubspaces();
//      M1_subspaces = M1_decomposed.size();
//
//      //sanity check it
//      if(M0_space->getDimension() >= M1_space->getDimension()){
//        std::cout << "Cannot reduce to a bigger cspace. We need dim(M0) < dim(M1)." << std::endl;
//        std::cout << "M0 dimension: " << M0_space->getDimension() << std::endl;
//        std::cout << "M1 dimension: " << M1_space->getDimension() << std::endl;
//        exit(0);
//      }
//
//      if(M0_subspaces < M1_subspaces){
//        //get C1
//        C1_subspaces = M1_subspaces - M0_subspaces;
//
//        if(C1_subspaces<=1){
//          StateSpacePtr C1_space = M1_decomposed.at(M0_subspaces);
//          C1 = std::make_shared<SpaceInformation>(C1_space);
//        }else{
//          StateSpacePtr C1_space = std::make_shared<ompl::base::CompoundStateSpace>();
//          for(uint k = M0_subspaces; k < M0_subspaces+C1_subspaces; k++){
//            C1_space->as<ob::CompoundStateSpace>()->addSubspace(M1_decomposed.at(k),1);
//          }
//          C1 = std::make_shared<SpaceInformation>(C1_space);
//        }
//      }else{
//        StateSpacePtr M1_back = M1_decomposed.back();
//        if((M1_back->getType() == base::STATE_SPACE_SO3) && (M0_space->getDimension() == 5)){
//          StateSpacePtr C1_space = (std::make_shared<ob::SO2StateSpace>());
//          C1 = std::make_shared<SpaceInformation>(C1_space);
//          C1_subspaces = 1;
//        }else{
//
//          if(M0_space->getDimension() < M1_space->getDimension()){
//            //try to fix it
//
//            StateSpacePtr M1_1_space = M1_decomposed.at(0);
//            if(M1_1_space->getDimension() == M0_space->getDimension()){
//              C1 = std::make_shared<SpaceInformation>(M1_decomposed.at(1));
//              C1_subspaces = 1;
//
//            }else{
//              std::cout << "M0 (dimension: " << M0_space->getDimension() << ") contains " << M0_subspaces << " subspaces of type " << M0_space->getName() << " " << M0_space->getType() << std::endl;
//              std::cout << "M1 (dimension: " << M1->getStateDimension() << ") contains " << M1_subspaces  << " subspaces of type " << M1->getStateSpace()->getName() << " " << M1->getStateSpace()->getType() << std::endl;
//              std::cout << "Subspaces do not match. We can only handle this in the case of S1xS1 subset SO(3)" << std::endl;
//              exit(0);
//            }
//
//
//          }else{
//            std::cout << "M0 (dimension: " << M0_space->getDimension() << ") contains " << M0_subspaces << " subspaces of type " << M0_space->getName() << " " << M0_space->getType() << std::endl;
//            std::cout << "M1 (dimension: " << M1->getStateDimension() << ") contains " << M1_subspaces  << " subspaces of type " << M1->getStateSpace()->getName() << " " << M1->getStateSpace()->getType() << std::endl;
//            std::cout << "Subspaces do not match. We can only handle this in the case of S1xS1 subset SO(3)" << std::endl;
//            exit(0);
//          }
//        }
//
//      }
//      C1_sampler = C1->allocStateSampler();
//
//    }
//    std::cout << "M" << id-1 << " (dimension: " << M0_space->getDimension() << ") contains " << M0_subspaces << " subspaces of type " << M0_space->getName() << " " << M0_space->getType() << std::endl;
//    std::cout << "M" << id << " (dimension: " << M1->getStateDimension() << ") contains " << M1_subspaces  << " subspaces of type " << M1->getStateSpace()->getName() << " " << M1->getStateSpace()->getType() << std::endl;
//    std::cout << "C" << id << " (dimension: " << C1->getStateDimension() << ") contains " << C1_subspaces  << " subspaces of type " << C1->getStateSpace()->getName() << " " << C1->getStateSpace()->getType() << std::endl;
//
//  }

}

PRMQuotient::~PRMQuotient()
{
}

void PRMQuotient::clear()
{
  PRMBasic::clear();
  isSampled = false; 
  //C1_sampler.reset();
      //C1_sampler = C1->allocStateSampler();
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
double PRMQuotient::getSamplingDensity()
{
  if(previous == nullptr){
    return (double)num_vertices(g_)/(double)M1->getSpaceMeasure();
  }else{
    //get graph length
    double Lprev = 0.0;
    og::PRMQuotient *PRMprevious = static_cast<og::PRMQuotient*>(previous);
    const Graph gprev = PRMprevious->getRoadmap();
    foreach (Edge e, boost::edges(gprev))
    {
      EdgeProperty ep = get(boost::edge_weight_t(), gprev, e);
      ob::Cost weight = ep.getCost();
      Lprev += weight.value();
    }
    return (double)num_vertices(g_)/(M1->getSpaceMeasure());
  }
}


void PRMQuotient::getPlannerData(base::PlannerData &data) const
{
  PRMBasic::getPlannerData(data);
}

bool PRMQuotient::Sample(ob::State *workState)
{
  if(previous == nullptr){
    //without any underlying CSpace, the behavior is equal to PRM
    return sampler_->sample(workState);
  }else{
    //Adjusted sampling function: Sampling in G0 x C1

    ob::SpaceInformationPtr M0 = previous->getSpaceInformation();
    base::State *s_C1 = C1->allocState();
    base::State *s_M0 = M0->allocState();

    C1_sampler->sampleUniform(s_C1);
    previous->SampleGraph(s_M0);
    mergeStates(s_M0, s_C1, workState);

    return M1->isValid(workState);
  }
}

bool PRMQuotient::SampleGraph(ob::State *workState)
{

  PDF<Edge> pdf = GetEdgePDF();
  if(pdf.empty()){
    std::cout << "cannot sample empty(?) graph" << std::endl;
    exit(0);
  }
  Edge e = pdf.sample(rng_.uniform01());
  double t = rng_.uniform01();

  const Vertex v1 = boost::source(e, g_);
  const Vertex v2 = boost::target(e, g_);
  const ob::State *from = stateProperty_[v1];
  const ob::State *to = stateProperty_[v2];

  M1->getStateSpace()->interpolate(from, to, t, workState);

  lastSourceVertexSampled = v1;
  lastTargetVertexSampled = v2;
  lastTSampled = t;

  isSampled = true;

  return true;

}

ompl::PDF<og::PRMBasic::Edge> PRMQuotient::GetEdgePDF()
{
  PDF<Edge> pdf;
  foreach (Edge e, boost::edges(g_))
  {
    ob::Cost weight = get(boost::edge_weight_t(), g_, e).getCost();
    pdf.add(e, weight.value());
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
