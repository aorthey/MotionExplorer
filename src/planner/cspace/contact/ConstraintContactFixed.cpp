#include <ompl/base/Constraint.h>
#include "planner/cspace/contact/ConstraintContactFixed.h"
#include "planner/cspace/cspace_geometric_contact.h"

ConstraintContactFixed::ConstraintContactFixed(
    GeometricCSpaceContact *cspace, int ambientSpaceDim, 
    int linkNumber, std::vector<Triangle3D> tris)
  : ConstraintContact(cspace, ambientSpaceDim, linkNumber, tris)
{
}

void ConstraintContactFixed::function(
    const Eigen::Ref<const Eigen::VectorXd> &x, 
    Eigen::Ref<Eigen::VectorXd> out) const
{
    if(getMode() == 0)
    {
      out[0]=0.0;
    }else{
      Vector3 contact = getPos(x);
      out[0] = contactDistanceToMesh(contact);
    }
}

int ConstraintContactFixed::getNumberOfModes()
{
  return 2;
}
