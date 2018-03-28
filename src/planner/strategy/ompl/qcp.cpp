#include "qcp.h"
#include "planner/cover/open_set_convex.h"
#include "elements/plannerdata_vertex_annotated.h"
#include "planner/validitychecker/validity_checker_simplicial_complex.h"
#include <ompl/util/Console.h>

using namespace og;
QCP::QCP(const ob::SpaceInformationPtr &si, Quotient *previous_):
  Quotient(si, previous_)
{
}

QCP::~QCP()
{
  cspace_cover.Clear();
}

void QCP::Init()
{
  if(const ob::State *st = pis_.nextStart()){
    startS_.push_back(st);
  }
  if(const ob::State *gl = pis_.nextGoal()){
    goalS_.push_back(gl);
  }
  if(startS_.empty()){
    std::cout << "No start states" << std::endl;
    exit(1);
  }
  if(goalS_.empty()){
    std::cout << "No goal states" << std::endl;
    exit(1);
  }

  auto checkerPtr = static_pointer_cast<ValidityCheckerSimplicialComplex>(si_->getStateValidityChecker());
  cspace_cover.Clear();

  const ob::State *start = startS_.at(0);
  cover::OpenSetConvex *start_region = checkerPtr->ComputeNeighborhood(start);
  cspace_cover.AddStartOpenSet(start_region);

  const ob::State *goal = goalS_.at(0);
  cover::OpenSetConvex *goal_region = checkerPtr->ComputeNeighborhood(goal);
  cspace_cover.AddGoalOpenSet(goal_region);
}

void QCP::Grow(double t)
{
}

void QCP::CheckForSolution(ob::PathPtr &solution)
{
}

void QCP::getPlannerData(ob::PlannerData &data) const
{
  auto checkerPtr = static_pointer_cast<ValidityCheckerSimplicialComplex>(si_->getStateValidityChecker());

  std::vector<cover::OpenSetConvex*> sets = cspace_cover.GetCover();
  for(uint k = 0; k < sets.size(); k++){
    const ob::State *s = sets.at(k)->GetCenter();
    PlannerDataVertexAnnotated v(s, 0);
    v.SetOpenSet(sets.at(k));
    if((int)k==cspace_cover.GetStartSetIndex())
      data.addStartVertex(v);
    else if((int)k==cspace_cover.GetGoalSetIndex())
      data.addGoalVertex(v);
    else 
      data.addVertex(v);
  }
}
