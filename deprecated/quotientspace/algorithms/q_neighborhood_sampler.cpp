#include "common.h"
#include "gui/common.h"
#include "q_neighborhood_sampler.h"

using namespace ompl::geometric;

//############################################################################
// Setup
//############################################################################
QNeighborhoodSampler::QNeighborhoodSampler(const base::SpaceInformationPtr &si, Quotient *parent ): BaseT(si, parent)
{
  setName("QNeighborhoodSampler"+std::to_string(id));
}

//############################################################################
// Grow Functions
//############################################################################
void QNeighborhoodSampler::Grow()
{
  if(firstRun){
    firstRun = false;
    Init();
  }
  Configuration *q = new Configuration(Q1);
  Q1_sampler->sampleUniform(q->state);
  q->SetRadius(DistanceInnerRobotToObstacle(q->state));
  if(q->GetRadius()>minimum_neighborhood_radius){
    AddConfigurationToCoverGraph(q);
  }
}
bool QNeighborhoodSampler::GetSolution(ob::PathPtr &solution)
{
  return false;
}
