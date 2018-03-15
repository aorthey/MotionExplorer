#include "planner/cspace/cspace_decorator.h"
#include "planner/validitychecker/validity_checker_ompl.h"

CSpaceOMPLDecorator::CSpaceOMPLDecorator(CSpaceOMPL* cspace_ompl_):
  CSpaceOMPL(cspace_ompl_->world, cspace_ompl_->robot_idx), cspace_ompl(cspace_ompl_)
{
  //input = cspace_ompl->input;
  //Nklampt = cspace_ompl->Nklampt;
  //Nompl = cspace_ompl->Nompl;
  //fixedBase = cspace_ompl->fixedBase;
}
const oc::StatePropagatorPtr CSpaceOMPLDecorator::StatePropagatorPtr(oc::SpaceInformationPtr si){
  return cspace_ompl->StatePropagatorPtr(si);
}
const ob::StateValidityCheckerPtr CSpaceOMPLDecorator::StateValidityCheckerPtr(){
  return StateValidityCheckerPtr(SpaceInformationPtr());
}
void CSpaceOMPLDecorator::ConfigToOMPLState(const Config &q, ob::State *qompl){
  return cspace_ompl->ConfigToOMPLState(q, qompl);
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
void CSpaceOMPLDecorator::print() const{
  cspace_ompl->print();
}
const ob::StateSpacePtr CSpaceOMPLDecorator::SpacePtr(){
  return cspace_ompl->SpacePtr();
}
const oc::RealVectorControlSpacePtr CSpaceOMPLDecorator::ControlSpacePtr(){
  return cspace_ompl->ControlSpacePtr();
}
uint CSpaceOMPLDecorator::GetDimensionality() const{
  return cspace_ompl->GetDimensionality();
}
uint CSpaceOMPLDecorator::GetControlDimensionality() const{
  return cspace_ompl->GetControlDimensionality();
}
void CSpaceOMPLDecorator::SetCSpaceInput(const CSpaceInput &input_){
  cspace_ompl->SetCSpaceInput(input_);
}
Robot* CSpaceOMPLDecorator::GetRobotPtr(){
  return cspace_ompl->GetRobotPtr();
}
CSpace* CSpaceOMPLDecorator::GetCSpacePtr(){
  return cspace_ompl->GetCSpacePtr();
}
ob::SpaceInformationPtr CSpaceOMPLDecorator::SpaceInformationPtr(){
  if(cspace_ompl->si==nullptr){
    cspace_ompl->si = std::make_shared<ob::SpaceInformation>(SpacePtr());
    const ob::StateValidityCheckerPtr checker = StateValidityCheckerPtr();
    cspace_ompl->si->setStateValidityChecker(checker);
  }
  return cspace_ompl->si;
}
Vector3 CSpaceOMPLDecorator::getXYZ(const ob::State* s){
  return cspace_ompl->getXYZ(s);
}
void CSpaceOMPLDecorator::print(std::ostream& out) const
{
  cspace_ompl->print(out);
}
const ob::StateValidityCheckerPtr CSpaceOMPLDecorator::StateValidityCheckerPtr(ob::SpaceInformationPtr si){
  return cspace_ompl->StateValidityCheckerPtr(si);
}

//#############################################################################

CSpaceOMPLDecoratorInnerOuter::CSpaceOMPLDecoratorInnerOuter(CSpaceOMPL *cspace_ompl_, CSpace *outer_):
  CSpaceOMPLDecorator(cspace_ompl_), outer(outer_)
{
}
const ob::StateValidityCheckerPtr CSpaceOMPLDecoratorInnerOuter::StateValidityCheckerPtr(ob::SpaceInformationPtr si){
  return std::make_shared<OMPLValidityCheckerInnerOuter>(si, this, cspace_ompl->GetCSpacePtr(), outer);
}

//#############################################################################
CSpaceOMPLDecoratorNecessarySufficient::CSpaceOMPLDecoratorNecessarySufficient(CSpaceOMPL *cspace_ompl_, uint robot_outer_idx_):
  CSpaceOMPLDecorator(cspace_ompl_), robot_outer_idx(robot_outer_idx_)
{
  outer = new SingleRobotCSpace(*cspace_ompl->world,robot_outer_idx,&cspace_ompl->worldsettings);
  // SingleRobotCSpace* cso = static_cast<SingleRobotCSpace*>(outer);
  // Robot *ro = cso->GetRobot();
  // std::cout << ro->name << std::endl;
  // exit(0);
}
const ob::StateValidityCheckerPtr CSpaceOMPLDecoratorNecessarySufficient::StateValidityCheckerPtr(ob::SpaceInformationPtr si){
  return std::make_shared<OMPLValidityCheckerNecessarySufficient>(si, cspace_ompl, outer);
}
