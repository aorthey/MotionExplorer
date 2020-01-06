#pragma once
#include "planner/cspace/cspace.h"
#include <ompl/base/spaces/SE3StateSpace.h>

//GeometricCSpaceOMPL: Space = Configuration manifold; control space = tangent
//space of configuration manifold, i.e. control happens in velocity space
//
class GeometricCSpaceOMPLSE3RN: public CSpaceOMPL
{
  public:
    GeometricCSpaceOMPLSE3RN(RobotWorld *world_, int robot_idx);

    virtual const oc::StatePropagatorPtr StatePropagatorPtr(oc::SpaceInformationPtr si) override;

    virtual void initSpace() override;
    virtual void ConfigToOMPLState(const Config &q, ob::State *qompl) override;
    virtual Config OMPLStateToConfig(const ob::State *qompl) override;
    Config OMPLStateToConfig(const ob::SE3StateSpace::StateType *qomplSE3, const ob::RealVectorStateSpace::StateType *qomplRnState);

    virtual void print(std::ostream& out = std::cout) const;
    virtual bool isDynamic() const override;
    virtual Vector3 getXYZ(const ob::State*) override;
};

