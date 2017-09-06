#include "cspace.h"

class GeometricCSpaceOMPLDecorator: public GeometricCSpaceOMPL
{
  public:
    GeometricCSpaceOMPLDecorator(GeometricCSpaceOMPL*);

    virtual const oc::StatePropagatorPtr StatePropagatorPtr(oc::SpaceInformationPtr si);
    virtual const ob::StateValidityCheckerPtr StateValidityCheckerPtr(ob::SpaceInformationPtr si);

    virtual void initSpace();
    virtual void initControlSpace();

    virtual ob::ScopedState<> ConfigToOMPLState(const Config &q);
    virtual ob::State* ConfigToOMPLStatePtr(const Config &q);

    virtual Config OMPLStateToConfig(const ob::ScopedState<> &qompl);
    virtual Config OMPLStateToConfig(const ob::State *qompl);

    Config OMPLStateToConfig(const ob::SE3StateSpace::StateType *qomplSE3, const ob::RealVectorStateSpace::StateType *qomplRnState);

    virtual void print();

  protected:
    GeometricCSpaceOMPL *geometric_cspace;
};

class GeometricCSpaceOMPLDecoratorInnerOuter: public GeometricCSpaceOMPLDecorator
{
  public:
    GeometricCSpaceOMPLDecoratorInnerOuter(GeometricCSpaceOMPL *geometric_cspace_, CSpace *outer_);
    virtual const ob::StateValidityCheckerPtr StateValidityCheckerPtr(ob::SpaceInformationPtr si);
  private:
    CSpace *outer;
};
