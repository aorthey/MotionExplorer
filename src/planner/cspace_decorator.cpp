#include "planner/cspace_decorator.h"
#include "planner/validity_checker_ompl.h"

CSpaceOMPLDecorator::CSpaceOMPLDecorator(CSpaceOMPL* cspace_ompl_):
  CSpaceOMPL(cspace_ompl_->GetRobotPtr(), cspace_ompl_->GetCSpacePtr())
{
  cspace_ompl = cspace_ompl_;

}
const oc::StatePropagatorPtr CSpaceOMPLDecorator::StatePropagatorPtr(oc::SpaceInformationPtr si){
  return cspace_ompl->StatePropagatorPtr(si);
}
const ob::StateValidityCheckerPtr CSpaceOMPLDecorator::StateValidityCheckerPtr(ob::SpaceInformationPtr si){
  return cspace_ompl->StateValidityCheckerPtr(si);
}
const ob::StateValidityCheckerPtr CSpaceOMPLDecorator::StateValidityCheckerPtr(oc::SpaceInformationPtr si){
  return cspace_ompl->StateValidityCheckerPtr(si);
}
ob::ScopedState<> CSpaceOMPLDecorator::ConfigToOMPLState(const Config &q){
  return cspace_ompl->ConfigToOMPLState(q);
}
Config CSpaceOMPLDecorator::OMPLStateToConfig(const ob::ScopedState<> &qompl){
  return cspace_ompl->OMPLStateToConfig(qompl);
}
Config CSpaceOMPLDecorator::OMPLStateToConfig(const ob::State *qompl){
  return cspace_ompl->OMPLStateToConfig(qompl);
}
void CSpaceOMPLDecorator::initSpace(){
  cspace_ompl->initSpace();
}
void CSpaceOMPLDecorator::initControlSpace(){
  cspace_ompl->initControlSpace();
}
void CSpaceOMPLDecorator::print(){
  cspace_ompl->print();
}
const ob::StateSpacePtr CSpaceOMPLDecorator::SpacePtr(){
  return cspace_ompl->SpacePtr();
}
const oc::RealVectorControlSpacePtr CSpaceOMPLDecorator::ControlSpacePtr(){
  return cspace_ompl->ControlSpacePtr();
}
uint CSpaceOMPLDecorator::GetDimensionality(){
  return cspace_ompl->GetDimensionality();
}
uint CSpaceOMPLDecorator::GetControlDimensionality(){
  return cspace_ompl->GetControlDimensionality();
}
void CSpaceOMPLDecorator::SetPlannerInput(PlannerInput &input_){
  cspace_ompl->SetPlannerInput(input_);
}
Robot* CSpaceOMPLDecorator::GetRobotPtr(){
  return cspace_ompl->GetRobotPtr();
}
CSpace* CSpaceOMPLDecorator::GetCSpacePtr(){
  return cspace_ompl->GetCSpacePtr();
}

//#############################################################################

CSpaceOMPLDecoratorInnerOuter::CSpaceOMPLDecoratorInnerOuter(CSpaceOMPL *cspace_ompl_, CSpace *outer_):
  CSpaceOMPLDecorator(cspace_ompl_), outer(outer_)
{
}
const ob::StateValidityCheckerPtr CSpaceOMPLDecoratorInnerOuter::StateValidityCheckerPtr(ob::SpaceInformationPtr si){
  return std::make_shared<OMPLValidityCheckerInnerOuter>(si, this, cspace_ompl->GetCSpacePtr(), outer);
}
