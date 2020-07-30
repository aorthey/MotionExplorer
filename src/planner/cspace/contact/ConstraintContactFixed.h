#pragma once
#include <ompl/base/Constraint.h>
#include "planner/cspace/cspace_geometric.h"
#include "planner/cspace/contact/ConstraintContact.h"
#include <ompl/base/spaces/constraint/ConstrainedStateSpace.h>
#include <ompl/util/RandomNumbers.h>

class GeometricCSpaceContact;

class ConstraintContactFixed : public ConstraintContact
{
  using BaseT = ConstraintContact;

public:
    ConstraintContactFixed(GeometricCSpaceContact *cspace, 
        int ambientSpaceDim, int linkNumber, 
        std::vector<Triangle3D> tris);

    virtual void function(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::VectorXd> out) const override;
    virtual int getNumberOfModes() override;

    virtual void setMode(int mode) override;

    void setFixedContactPoint(const Vector3&);
    const Vector3& getFixedContactPoint() const;

private:
    Vector3 fixedContactPoint_;
    bool hasFixedContactPoint_{false};
};
