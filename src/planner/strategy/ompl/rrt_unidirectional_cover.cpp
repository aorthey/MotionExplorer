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
  uint attempts = 0;
  uint max_attempts = 10;

  while(!found){
    attempts++;
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
    if(attempts>max_attempts){
      //std::cout << "WARNING: " << attempts << " attempts." << std::endl;
      break;
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

bool RRTUnidirectionalCover::ConnectedToGoal(Configuration* q)
{
  if(IsInsideCover(q_goal)){
    hasSolution = true;
    Configuration *qn = G_->nearest(q_goal);
    q_goal->parent = qn;
    G_->add(q_goal);
    return true;
  }
  return false;
}

void RRTUnidirectionalCover::CheckForSolution(ob::PathPtr &solution)
{
  if(!hasSolution) return;

  if (q_goal != nullptr){

    std::vector<Configuration *> q_path;
    while (q_goal != nullptr){
      q_path.push_back(q_goal);
      q_goal = q_goal->parent;
    }

    auto path(std::make_shared<PathGeometric>(si_));
    for (int i = q_path.size() - 1; i >= 0; --i){
      path->append(q_path[i]->state);
    }
    solution = path;
    goalBias_ = 0.0;
  }
}

RRTUnidirectional::Configuration* RRTUnidirectionalCover::Connect(Configuration *q_near, Configuration *q_random)
{
  //##############################################################################
  // Move towards q_random until the boundary of our cover is breached
  //##############################################################################

  double d = si_->distance(q_near->state, q_random->state);
  double maxD = q_near->GetRadius();
  if(d > maxD)
  {
    //maxD/d -> t \in [0,1] on boundary
    si_->getStateSpace()->interpolate(q_near->state, q_random->state, maxD / d, q_random->state);
  }

  //##############################################################################
  //q_new is a point on the outer boundary of our cover. We do not need
  //to check the motion towards it. It is valid because the edge lies inside
  //Cfree
  //##############################################################################
  auto *q_new = new Configuration(si_);
  si_->copyState(q_new->state, q_random->state);
  q_new->parent = q_near;

  auto checkerPtr = static_pointer_cast<OMPLValidityChecker>(si_->getStateValidityChecker());
  double d_new = checkerPtr->Distance(q_new->state);
  q_new->openset = new cover::OpenSetHypersphere(si_, q_new->state, d_new);

  G_->add(q_new);
}

bool RRTUnidirectionalCover::SampleGraph(ob::State *q_random_graph)
{
  PDF<Configuration*> pdf = GetConfigurationPDF();

  //##############################################################################
  auto checkerPtr = static_pointer_cast<OMPLValidityCheckerNecessarySufficient>(si_->getStateValidityChecker());
  //##############################################################################
  uint attempts = 0;
  bool foundNecessary = false;
  Configuration *q;
  while(!foundNecessary)
  {
    q = pdf.sample(rng_.uniform01());
    double t = rng_.uniform01();

    const ob::State *q_from = q->state;
    const ob::State *q_to = q->parent->state;
    M1->getStateSpace()->interpolate(q_from, q_to, t, q_random_graph);

    if(!checkerPtr->IsSufficient(q_random_graph)){
      foundNecessary = true;
    }
    if(++attempts > 5){
      break;
    }
  }

  if(!foundNecessary) return false;

  double d = q->openset->GetRadius();
  //sampler_->sampleGaussian(q_random_graph, q_random_graph, d);
  sampler_->sampleUniformNear(q_random_graph, q_random_graph, d);
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
      //pdf.add(configuration, 1.0/configuration->openset->GetRadius());
      pdf.add(configuration, exp(-configuration->openset->GetRadius()));
    }
  }

  if(pdf.empty()){
    std::cout << "cannot sample empty(?) graph" << std::endl;
    exit(0);
  }
  return pdf;
}


// bool RRTUnidirectionalCover::checkMotion(Configuration *q1, Configuration *q2) 
// {
//   ob::State *s1 = q1->state;
//   ob::State *s2 = q2->state;

//   double d1 = q1->GetRadius();
//   double d2 = q2->GetRadius();

//   auto checkerPtr = static_pointer_cast<OMPLValidityChecker>(si_->getStateValidityChecker());

//   if (!si_->isValid(s2))
//   {
//     return false;
//   }

//   double d12 = Distance(q1, q2);
//   const double epsilon = 1e-3;

//   bool result = true;
//   ob::State *s_next = si_->allocState();
//   double d = d1/d12;
//   uint iters = 0;
//   while(d<1){
//     iters++;
//     si_->getStateSpace()->interpolate(s1, s2, d, s_next);
//     if(!si_->isValid(s_next)){
//       result = false;
//       break;
//     }else{
//       double dn = checkerPtr->Distance(s_next);
//       if(dn<epsilon)
//       {
//         result =false;
//         break;
//       }
//       d += dn/d12;
//     }
//   }
//   si_->freeState(s_next);
//   return result;
// }

  ////##############################################################################
  //Configuration *q = pdf.sample(rng_.uniform01());
  //double t = rng_.uniform01();

  //const ob::State *q_from = q->state;
  //const ob::State *q_to = q->parent->state;
  //M1->getStateSpace()->interpolate(q_from, q_to, t, q_random_graph);

  //double d = q->openset->GetRadius();
  //sampler_->sampleGaussian(q_random_graph, q_random_graph, d);
  ////sampler_->sampleUniformNear(q_random_graph, q_random_graph, d);
  ////##############################################################################
