#include "planner/strategy/strategy_output.h"
#include <ompl/multilevel/datastructures/PlannerDataVertexAnnotated.h>
#include <ompl/base/PlannerData.h>
#include "elements/roadmap.h"
#include "common.h"
#include <ompl/control/PathControl.h>
#include <ompl/geometric/PathSimplifier.h>

namespace om = ompl::multilevel;

StrategyOutput::StrategyOutput(std::vector<CSpaceOMPL*> cspace_levels):
  cspace_levels_(cspace_levels)
{
  pathVec_.resize(cspace_levels.size(), nullptr);
}

void StrategyOutput::SetPlannerData( ob::PlannerDataPtr pd)
{
  plannerData_ = pd;
  plannerData_->decoupleFromPlanner();
  plannerData_->computeEdgeWeights();
  roadmap_ = std::make_shared<Roadmap>(plannerData_, cspace_levels_);
}

void StrategyOutput::SetProblemDefinition( ob::ProblemDefinitionPtr pdef )
{
  pdefVec_.push_back(pdef);
}

PathPiecewiseLinear* StrategyOutput::getSolutionPath(int level)
{
  ob::ProblemDefinitionPtr pdef = pdefVec_.at(level);
  if(pdef->hasExactSolution() || pdef->hasApproximateSolution())
  {
      pathVec_.at(level) = new PathPiecewiseLinear(pdef->getSolutionPath(), 
          cspace_levels_.back(), cspace_levels_.at(level));
      return pathVec_.at(level);
  }else{
      return nullptr;
  }
}

void StrategyOutput::SetProblemDefinition( std::vector<ob::ProblemDefinitionPtr> pdefVec )
{
  pdefVec_ = pdefVec;
}

bool StrategyOutput::hasExactSolution(){
  return pdefVec_.back()->hasExactSolution();
}

bool StrategyOutput::hasApproximateSolution(){
  for(uint k = 0; k < pdefVec_.size(); k++)
  {
    if(pdefVec_.at(k)->hasApproximateSolution())
    {
      return true;
    }
  }
  return false;
}

ob::PlannerDataPtr StrategyOutput::GetPlannerDataPtr(){
  return plannerData_;
}

void StrategyOutput::DrawGL(GUIState& state, int level)
{
  if(roadmap_ != nullptr)
  {
      roadmap_->DrawGL(state, level);
  }
}
void StrategyOutput::DrawGLPath(GUIState& state, int level)
{
  std::cout << "DRAWGL " << level << std::endl;
    // pathVec_.at(level) = getSolutionPath(level);
    pathVec_.at(level)->DrawGL(state);
}

std::ostream& operator<< (std::ostream& out, const StrategyOutput& so) 
{
  out << std::string(80, '-') << std::endl;
  out << "Planning Output" << std::endl;
  out << std::string(80, '-') << std::endl;
  out << " robot                : " << so.cspace_levels_.back()->GetName() << std::endl;
  if(so.plannerData_){
    out << " roadmap vertices     : " << so.plannerData_->numVertices() << std::endl;
    out << " roadmap edges        : " << so.plannerData_->numEdges() << std::endl;
  }
  out << " planner time         : " << so.planner_time << std::endl;
  out << " max planner time     : " << so.max_planner_time << std::endl;
  out << std::string(80, '-') << std::endl;
  return out;
}
