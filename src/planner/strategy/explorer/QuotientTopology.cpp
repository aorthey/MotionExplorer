#include "QuotientTopology.h"
#include <ompl/tools/config/SelfConfig.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>

using namespace og;
using namespace ob;
#define foreach BOOST_FOREACH

QuotientTopology::QuotientTopology(const ob::SpaceInformationPtr &si, Quotient *parent_ ):
  BaseT(si, parent_)
{
  setName("QuotientTopology"+std::to_string(id));
  Planner::declareParam<double>("range", this, &QuotientTopology::setRange, &QuotientTopology::getRange, "0.:1.:10000.");
  Planner::declareParam<double>("goal_bias", this, &QuotientTopology::setGoalBias, &QuotientTopology::getGoalBias, "0.:.1:1.");
  q_random = new Configuration(Q1);
}

QuotientTopology::~QuotientTopology()
{
  DeleteConfiguration(q_random);
}

void QuotientTopology::setGoalBias(double goalBias_)
{
  goalBias = goalBias_;
}
double QuotientTopology::getGoalBias() const
{
  return goalBias;
}
void QuotientTopology::setRange(double maxDistance_)
{
  maxDistance = maxDistance_;
}
double QuotientTopology::getRange() const
{
  return maxDistance;
}

void QuotientTopology::setup()
{
  BaseT::setup();
  ompl::tools::SelfConfig sc(Q1, getName());
  sc.configurePlannerRange(maxDistance);
  goal = pdef_->getGoal().get();
}
void QuotientTopology::clear()
{
  BaseT::clear();
}

bool QuotientTopology::GetSolution(ob::PathPtr &solution)
{
  if(hasSolution){
    return BaseT::GetSolution(solution);
  }else{
    return false;
  }
}

void QuotientTopology::Grow(){
  if(firstRun){
    Init();
    firstRun = false;
  }

  if(hasSolution){
    //No Goal Biasing if we already found a solution on this quotient space
    Sample(q_random->state);
  }else{
    double s = rng_.uniform01();
    if(s < goalBias){
      Q1->copyState(q_random->state, q_goal->state);
    }else{
      Sample(q_random->state);
    }
  }

  const Configuration *q_nearest = Nearest(q_random);
  double d = Q1->distance(q_nearest->state, q_random->state);
  if(d > maxDistance){
    Q1->getStateSpace()->interpolate(q_nearest->state, q_random->state, maxDistance / d, q_random->state);
  }

  totalNumberOfSamples++;
  if(Q1->checkMotion(q_nearest->state, q_random->state))
  {
    totalNumberOfFeasibleSamples++;
    Configuration *q_next = new Configuration(Q1, q_random->state);
    Vertex v_next = AddConfiguration(q_next);
    AddEdge(q_nearest->index, v_next);

    if(!hasSolution){
      bool satisfied = sameComponentSparse(v_start_sparse, v_goal_sparse);
      if(satisfied)
      {
        hasSolution = true;
        // std::cout << std::string(80, '*') << std::endl;
        // std::cout << getName() << " Found Solution " << std::endl;
        // std::cout << std::string(80, '*') << std::endl;
        // uint startComponent = disjointSets_.find_set(v_start_sparse);
        // uint goalComponent = disjointSets_.find_set(v_goal_sparse);
        // std::cout << "start component:" << startComponent << "| goalComponent:" << goalComponent << std::endl;
        // EIterator it, end;
        // std::cout << "start vertex:" << v_start_sparse << "| goal vertex:" << v_goal_sparse << std::endl;
        // for(tie(it, end) = boost::edges(graphSparse_); it != end; ++it)
        // {
        //     const Vertex v1 = boost::source(*it, graphSparse_);
        //     const Vertex v2 = boost::target(*it, graphSparse_);
        //     std::cout << v1 << "-" << v2 << std::endl;
        // }
      }
    }
  }
}

