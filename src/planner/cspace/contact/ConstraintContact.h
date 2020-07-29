#pragma once
#include <ompl/base/Constraint.h>
#include "planner/cspace/cspace_geometric.h"
#include "planner/cspace/contact/ConstraintMultiMode.h"
#include <ompl/base/spaces/constraint/ConstrainedStateSpace.h>

class GeometricCSpaceContact;

class ConstraintContact : public ConstraintMultiMode
{

public:
    ConstraintContact(
        GeometricCSpaceContact *cspace, 
        int ambientSpaceDim,
        int linkNumber,
        std::vector<Triangle3D> tris);

    Vector3 getPos(const Eigen::Ref<const Eigen::VectorXd> &xd) const;

    const std::vector<Triangle3D>& getTriangles() const;

    double contactDistanceToMesh(const Vector3& contact) const;

private:
    GeometricCSpaceContact *cspace_;
    Robot *robot_;
    int linkNumber_;
    std::vector<Triangle3D> tris_;
    Vector3 zero_;
};
