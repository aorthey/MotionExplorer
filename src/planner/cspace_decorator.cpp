#include "planner/cspace_decorator.h"
#include "planner/validity_checker_ompl.h"

GeometricCSpaceOMPLDecorator::GeometricCSpaceOMPLDecorator(GeometricCSpaceOMPL* geometric_cspace_)
{
  geometric_cspace = geometric_cspace_;
}
const oc::StatePropagatorPtr GeometricCSpaceOMPLDecorator::StatePropagatorPtr(oc::SpaceInformationPtr si){
  return geometric_cspace->StatePropagatorPtr(si);
}
const ob::StateValidityCheckerPtr GeometricCSpaceOMPLDecorator::StateValidityCheckerPtr(ob::SpaceInformationPtr si){
  return geometric_cspace->StateValidityCheckerPtr(si);
}
void GeometricCSpaceOMPLDecorator::initSpace(){
  geometric_cspace->initSpace();
}
void GeometricCSpaceOMPLDecorator::initControlSpace(){
  geometric_cspace->initControlSpace();
}
ob::ScopedState<> GeometricCSpaceOMPLDecorator::ConfigToOMPLState(const Config &q){
  return geometric_cspace->ConfigToOMPLState(q);
}
ob::State* GeometricCSpaceOMPLDecorator::ConfigToOMPLStatePtr(const Config &q){
  return geometric_cspace->ConfigToOMPLStatePtr(q);
}
Config GeometricCSpaceOMPLDecorator::OMPLStateToConfig(const ob::ScopedState<> &qompl){
  return geometric_cspace->OMPLStateToConfig(qompl);
}
Config GeometricCSpaceOMPLDecorator::OMPLStateToConfig(const ob::State *qompl){
  return geometric_cspace->OMPLStateToConfig(qompl);
}
Config GeometricCSpaceOMPLDecorator::OMPLStateToConfig(const ob::SE3StateSpace::StateType *qomplSE3, const ob::RealVectorStateSpace::StateType *qomplRnState){
  return geometric_cspace->OMPLStateToConfig(qomplSE3, qomplRnState);
}
void GeometricCSpaceOMPLDecorator::print(){
  geometric_cspace->print();
}
GeometricCSpaceOMPLDecoratorInnerOuter::GeometricCSpaceOMPLDecoratorInnerOuter(GeometricCSpaceOMPL *geometric_cspace_, CSpace *outer_):
  GeometricCSpaceOMPLDecorator(geometric_cspace_), outer(outer_)
{
}
const ob::StateValidityCheckerPtr GeometricCSpaceOMPLDecoratorInnerOuter::StateValidityCheckerPtr(ob::SpaceInformationPtr si){
  return std::make_shared<OMPLValidityCheckerInnerOuter>(si, this, geometric_cspace->GetCSpacePtr(), outer);
}
