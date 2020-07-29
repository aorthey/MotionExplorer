#include <ompl/base/Constraint.h>
#include "planner/cspace/cspace_geometric_contact.h"
#include "planner/cspace/contact/ConstraintContact.h"

ConstraintContact::ConstraintContact(GeometricCSpaceContact *cspace, 
    int ambientSpaceDim,
    int linkNumber,
    std::vector<Triangle3D> tris):
  ConstraintMultiMode(ambientSpaceDim),
  cspace_(cspace),
  robot_(cspace->GetRobotPtr()),
  linkNumber_(linkNumber),
  tris_(tris)
{
  zero_.setZero();
}

const std::vector<Triangle3D>& ConstraintContact::getTriangles() const
{
  return tris_;
}

Vector3 ConstraintContact::getPos(const Eigen::Ref<const Eigen::VectorXd> &xd) const
{
    /**
     * Member function of class FixedContactConstraint2D:
     * Returns position of given robot link in world coordinates.
     */

    if (xd!=xd){
        std::cout << std::string (80, '-') << std::endl;
        std::cout << "EigenVector: " << std::endl;
        std::cout << xd << std::endl;
        exit(0);
    }

    Config q = cspace_->EigenVectorToConfig(xd);

    robot_->UpdateConfig(q);
    robot_->UpdateGeometry();

    Vector3 v;
    robot_->GetWorldPosition(zero_, linkNumber_, v);

    return v;
}

double ConstraintContact::contactDistanceToMesh(const Vector3& contact) const
{
    Vector3 closestPt;
    Real distances = dInf;

    //loop over all surface triangles to get triangle that's closest
    for (uint j = 0; j < getTriangles().size(); j++) {
        Vector3 cP = getTriangles().at(j).closestPoint(contact);
        Real d = contact.distance(cP);

        if (distances > d){
            distances = d;
            closestPt = cP;
        }
    }

    Real distVect = contact.distance(closestPt);
    return distVect;
}

//void ConstraintContact::function(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::VectorXd> out) const
//{
//    // ---------- Contact with chosen Link and chosen Obstacle ------------------
//    Vector3 contact = getPos(x);
//    Vector3 closestPt;
//    Real distances = 1000;

//    //loop over all surface triangles to get triangle that's closest
//    for (uint j = 0; j < getTriangles().size(); j++) {
//        Vector3 cP = getTriangles().at(j).closestPoint(contact);
//        Real d = contact.distance(cP);

//        if (distances > d){
//            distances = d;
//            closestPt = cP;
//        }
//    }

//    Real distVect = contact.distance(closestPt);
//    out[0] = distVect;
//}
