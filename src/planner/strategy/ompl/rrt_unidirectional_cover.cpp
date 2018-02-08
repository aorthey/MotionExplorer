#include "rrt_unidirectional_cover.h"
#include "planner/validitychecker/validity_checker_ompl.h"

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
RRTUnidirectional::Configuration* RRTUnidirectionalCover::Connect(Configuration *q_near, Configuration *q_random)
{
  //##############################################################################
  // q_new_state <- BALL_maxDistance_(q_near) in direction of q_random
  //##############################################################################
  //double d = si_->distance(q_near->state, q_random->state);
  //if(d > maxDistance_){
  //  si_->getStateSpace()->interpolate(q_near->state, q_random->state, maxDistance_ / d, q_random->state);
  //}

  //##############################################################################
  // extend the tree from q_near towards q_new
  //##############################################################################
  if(si_->checkMotion(q_near->state, q_random->state)){

    auto *q_new = new Configuration(si_);
    si_->copyState(q_new->state, q_random->state);
    q_new->parent = q_near;

    //std::cout << "parent: " << std::endl;
    //si_->printState(q_new->parent->state);
    //std::cout << "new: " << std::endl;
    //si_->printState(q_new->state);

    auto checkerPtr = static_pointer_cast<OMPLValidityChecker>(si_->getStateValidityChecker());
    double d2 = checkerPtr->Distance(q_new->state);
    q_new->openset = new cover::OpenSetHypersphere(si_, q_new->state, d2 + deltaCoverPenetration_);

    G_->add(q_new);
    return q_new;

  }

  return nullptr;
}

bool RRTUnidirectionalCover::SampleGraph(ob::State *q_random_graph)
{
  PDF<Configuration*> pdf = GetConfigurationPDF();

  Configuration *q = pdf.sample(rng_.uniform01());
  double t = rng_.uniform01();

  const ob::State *q_from = q->state;
  const ob::State *q_to = q->parent->state;
  M1->getStateSpace()->interpolate(q_from, q_to, t, q_random_graph);

  double d = q->openset->GetRadius();
  //double d = 0.1;
  sampler_->sampleGaussian(q_random_graph, q_random_graph, d);

  //sampler_->sampleUniformNear(q_random_graph, q_random_graph, d);

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
      pdf.add(configuration, 1.0/configuration->openset->GetRadius());
    }
    //pdf.add(configuration, 1);
  }

  if(pdf.empty()){
    std::cout << "cannot sample empty(?) graph" << std::endl;
    exit(0);
  }
  return pdf;
}
