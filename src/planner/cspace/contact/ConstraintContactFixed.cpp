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
      if(!hasFixedContactPoint_)
      {
        out[0] = contactDistanceToMesh(contact);
      }else{
        out[0] = contactDistanceToPoint(contact, fixedContactPoint_);
      }
    }
}
void ConstraintContactFixed::setMode(int mode)
{
  BaseT::setMode(mode);
  hasFixedContactPoint_ = false;
}
void ConstraintContactFixed::setFixedContactPoint(const Vector3 &v)
{
  hasFixedContactPoint_ = true;
  fixedContactPoint_ = v;
}

const Vector3& ConstraintContactFixed::getFixedContactPoint() const
{
  return fixedContactPoint_;
}

int ConstraintContactFixed::getNumberOfModes()
{
  return 2;
}
