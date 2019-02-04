#include "common.h"
#include "gui/common.h"
#include "qsampler.h"

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
bool QSampler::GetSolution(ob::PathPtr &solution)
{
  return false;
}
