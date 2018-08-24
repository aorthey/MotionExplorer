#include "rrt_unidirectional_cover.h"
#include "elements/plannerdata_vertex_annotated.h"
#include "planner/cspace/validitychecker/validity_checker_ompl.h"

using namespace ompl::geometric;

RRTUnidirectionalCover::RRTUnidirectionalCover(const base::SpaceInformationPtr &si, og::Quotient *parent ): RRTUnidirectional(si, parent)
{
  setName("RRTUniCover");
}

RRTUnidirectionalCover::~RRTUnidirectionalCover(void)
{
}
void RRTUnidirectionalCover::getPlannerData(base::PlannerData &data) const
{
  std::vector<Configuration *> vertices;

  if (G_){
    G_->list(vertices);
  }

  data.addGoalVertex(PlannerDataVertexAnnotated(q_goal->state, 0, q_goal->openset->GetRadius()));

  for (auto &vertex : vertices)
  {
    double d = vertex->openset->GetRadius();
    if (vertex->parent == nullptr){
      data.addStartVertex(PlannerDataVertexAnnotated(vertex->state, 0, d));
    }else{
      double dp = vertex->parent->openset->GetRadius();
      data.addEdge(PlannerDataVertexAnnotated(vertex->parent->state, 0, dp), PlannerDataVertexAnnotated(vertex->state, 0, d));
    }
    if(!vertex->state){
      std::cout << "vertex state does not exists" << std::endl;
      si_->printState(vertex->state);
      exit(0);
    }
  }
}

void RRTUnidirectionalCover::Sample(RRTUnidirectional::Configuration *q)
{
  bool found = false;
  uint attempts = 0;
  uint max_attempts = 5;

  while(!found){
    attempts++;
    if(parent == nullptr){
      if(!hasSolution && rng_.uniform01() < goalBias_){
        goal->sampleGoal(q->state);
        found = true;
      }else{
        Q1_sampler->sampleUniform(q->state);
      }
    }else{
      if(!hasSolution && rng_.uniform01() < goalBias_){
        goal->sampleGoal(q->state);
        found = true;
      }else{
        ob::SpaceInformationPtr Q0 = parent->getSpaceInformation();
        base::State *s_X1 = X1->allocState();
        base::State *s_Q0 = Q0->allocState();

        X1_sampler->sampleUniform(s_X1);
        parent->SampleGraph(s_Q0);
        mergeStates(s_Q0, s_X1, q->state);

        X1->freeState(s_X1);
        Q0->freeState(s_Q0);
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
  if(q_goal == nullptr){
    std::cout << "(1)STOP, qgoal is nullptr" << std::endl;
    exit(0);
  }
  if(IsInsideCover(q_goal)){
    hasSolution = true;
    Configuration *qn = G_->nearest(q_goal);
    q_goal->parent = qn;
    G_->add(q_goal);
    if(q_goal == nullptr){
      std::cout << "(2)STOP, qgoal is nullptr" << std::endl;
      exit(0);
    }
    return true;
  }
  return false;
}

void RRTUnidirectionalCover::CheckForSolution(ob::PathPtr &solution)
{
  if(!hasSolution) return;

  if (q_goal->parent != nullptr){

    std::vector<Configuration *> q_path;
    Configuration *path_ptr = q_goal;
    while (path_ptr != nullptr){
      q_path.push_back(path_ptr);
      path_ptr = path_ptr->parent;
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
  q_new->openset = new cover::OpenSetHypersphere(checkerPtr->GetCSpaceOMPLPtr(), q_new->state, d_new);

  G_->add(q_new);
  return q_new;
}

bool RRTUnidirectionalCover::SampleGraph(ob::State *q_random_graph)
{
  PDF<Configuration*> pdf = GetConfigurationPDF();

  //##############################################################################
  auto checkerPtr = static_pointer_cast<OMPLValidityCheckerNecessarySufficient>(si_->getStateValidityChecker());
  //##############################################################################
  Configuration *q;

  q = pdf.sample(rng_.uniform01());
  double t = rng_.uniform01();

  const ob::State *q_from = q->state;
  const ob::State *q_to = q->parent->state;
  Q1->getStateSpace()->interpolate(q_from, q_to, t, q_random_graph);


  double d = q->openset->GetRadius();

  Q1_sampler->sampleUniformNear(q_random_graph, q_random_graph, d);
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
      //pdf.add(configuration, exp(-configuration->openset->GetRadius()));
      //pdf.add(configuration, d);
      //double d = configuration->openset->GetRadius();
      pdf.add(configuration, 1.0);
      //pdf.add(configuration, 1.0/d);
      //pdf.add(configuration, exp(-d));
    }
  }

  if(pdf.empty()){
    std::cout << "cannot sample empty(?) graph" << std::endl;
    exit(0);
  }
  return pdf;
}
