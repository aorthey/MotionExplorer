#include "common.h"
#include "gui/common.h"
#include "qsampler.h"
#include "elements/plannerdata_vertex_annotated.h"
#include <boost/foreach.hpp>

#define foreach BOOST_FOREACH

using namespace ompl::geometric;

//############################################################################
// Setup
//############################################################################
QSampler::QSampler(const base::SpaceInformationPtr &si, Quotient *parent ): BaseT(si, parent)
{
  setName("QSampler"+std::to_string(id));
}

//############################################################################
// Grow Functions
//############################################################################
void QSampler::Grow(double t)
{
  if(firstRun){
    if(const ob::State *st = pis_.nextStart()){
      if (st != nullptr){
        q_start = new Configuration(Q1, st);
        q_start->isStart = true;
        v_start = AddConfiguration(q_start);
      }
    }
    if (q_start == nullptr){
      OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
      exit(0);
    }

    if(const ob::State *st = pis_.nextGoal()){
      if (st != nullptr){
        q_goal = new Configuration(Q1, st);
        q_goal->isGoal = true;
        v_goal = AddConfiguration(q_goal);
      }
    }
    if (q_goal == nullptr){
      OMPL_ERROR("%s: There are no valid goal states!", getName().c_str());
      exit(0);
    }
    firstRun = false;
  }

  Configuration *q = new Configuration(Q1);
  Q1_sampler->sampleUniform(q->state);
  if(Q1->isValid(q->state)){
    q->isFeasible = true;
  }else{
    q->isFeasible = false;
  }
  AddConfiguration(q);
}
bool QSampler::GetSolution(ob::PathPtr &solution)
{
  return false;
}
void QSampler::getPlannerData(ob::PlannerData &data) const
{
  PlannerDataVertexAnnotated pstart(G[v_start]->state);
  data.addStartVertex(pstart);
  PlannerDataVertexAnnotated pgoal(G[v_goal]->state);
  data.addGoalVertex(pgoal);

  uint ctr = 0;
  foreach (const Vertex v, boost::vertices(G))
  {
    if(G[v]->isStart || G[v]->isGoal) continue;
    PlannerDataVertexAnnotated p(G[v]->state);
    if(G[v]->isFeasible){
      ctr++;
      p.SetFeasible();
    }else{
      p.SetInfeasible();
    }
    data.addVertex(p);

  }
  std::cout << "Sampled " << data.numVertices() << " vertices (" << ctr+2 << " feasible)." << std::endl;

  //write to file
}
