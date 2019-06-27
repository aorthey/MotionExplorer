#include "QuotientTopology.h"
#include <ompl/tools/config/SelfConfig.h>

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
    bool baset_sol = BaseT::GetSolution(solution);
    if(baset_sol){
      shortestPathVertices = shortestVertexPath_;
    }
    return baset_sol;
  }else{
    return false;
  }
}

void QuotientTopology::Grow(double t){
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
    v_last_added = AddConfiguration(q_next);
    AddEdge(q_nearest->index, v_last_added);

    double dist = 0.0;
    if(!hasSolution){
      bool satisfied = goal->isSatisfied(q_next->state, &dist);
      if(satisfied)
      {
        v_goal = AddConfiguration(q_goal);
        AddEdge(q_nearest->index, v_goal);
        hasSolution = true;
      }
    }else{
      Rewire(v_last_added);
    }
  }
}

double QuotientTopology::GetImportance() const{
  double N = (double)GetNumberOfVertices();
  return 1.0/(N+1);
}

//Make it faster by removing the valid check
bool QuotientTopology::Sample(ob::State *q_random)
{
  if(parent == nullptr){
    Q1_sampler->sampleUniform(q_random);
  }else{
    if(X1_dimension>0)
    {
      X1_sampler->sampleUniform(s_X1_tmp);
      parent->SampleQuotient(s_Q0_tmp);
      MergeStates(s_Q0_tmp, s_X1_tmp, q_random);
    }else{
      parent->SampleQuotient(q_random);
    }
  }
  return true;
}

bool QuotientTopology::SampleQuotient(ob::State *q_random_graph)
{
  const Vertex v = boost::random_vertex(G, rng_boost);
  Q1->getStateSpace()->copyState(q_random_graph, G[v]->state);
  return true;
}
