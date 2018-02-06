#include "rrt_unidirectional_cover.h"

using namespace ompl::geometric;

RRTUnidirectionalCover::RRTUnidirectionalCover(const base::SpaceInformationPtr &si, og::Quotient *previous ): RRTUnidirectional(si, previous)
{
  setName("RRTUniCover");
}

RRTUnidirectionalCover::~RRTUnidirectionalCover(void)
{
}

void RRTUnidirectionalCover::Sample(RRTUnidirectional::Configuration *q)
{
  bool found = false;
  while(!found){
    if(previous == nullptr){
      if(!hasSolution && rng_.uniform01() < goalBias_){
        goal->sampleGoal(q->state);
        found = true;
      }else{
        sampler_->sampleUniform(q->state);
      }
    }else{
      if(!hasSolution && rng_.uniform01() < goalBias_){
        goal->sampleGoal(q->state);
        found = true;
      }else{
        ob::SpaceInformationPtr M0 = previous->getSpaceInformation();
        base::State *s_C1 = C1->allocState();
        base::State *s_M0 = M0->allocState();

        C1_sampler->sampleUniform(s_C1);
        previous->SampleGraph(s_M0);
        mergeStates(s_M0, s_C1, q->state);

        C1->freeState(s_C1);
        M0->freeState(s_M0);
      }
    }
    if(!IsInsideCover(q)){
      found = true;
    }
  }
}

bool RRTUnidirectionalCover::IsInsideCover(Configuration* q)
{
  Configuration *qn = G_->nearest(q);
  NearestNeighbors<Configuration *>::DistanceFunction df = G_->getDistanceFunction();
  double d = df(q,qn);
  return d<=0;
}

bool RRTUnidirectionalCover::SampleGraph(ob::State *q_random_graph)
{
  PDF<Configuration*> pdf = GetConfigurationPDF();

  Configuration *q = pdf.sample(rng_.uniform01());

  const ob::State *q_center = q->state;

  double d = q->openset->GetRadius();

  sampler_->sampleGaussian(q_random_graph, q_center, d);
  //sampler_->sampleUniformNear(q_random_graph, q_center, d);

  return true;
}

ompl::PDF<RRTUnidirectional::Configuration*> RRTUnidirectionalCover::GetConfigurationPDF()
{
  PDF<Configuration*> pdf;
  std::vector<Configuration *> configurations;
  if(G_){
    G_->list(configurations);
  }
  for (auto &configuration : configurations)
  {
    if(!(configuration->parent == nullptr)){
      pdf.add(configuration, configuration->openset->GetRadius());
    }
  }

  if(pdf.empty()){
    std::cout << "cannot sample empty(?) graph" << std::endl;
    exit(0);
  }
  return pdf;
}
