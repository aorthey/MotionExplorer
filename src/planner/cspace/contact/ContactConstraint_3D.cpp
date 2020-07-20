#include <ompl/base/Constraint.h>
#include "planner/cspace/contact/ContactConstraint_3D.h"
#include "planner/cspace/cspace_geometric_R3_CONTACT.h"


ContactConstraint_3D::ContactConstraint_3D(GeometricCSpaceOMPLRCONTACT_3D *cspace, int ambientSpaceDim, Robot *robot, RobotWorld *world, uint linkNumber):
ob::Constraint(ambientSpaceDim, 2)  // (x,y,z, theta at 1st link,phi at 2nd)
, cspace_(cspace)
, robot_(robot)
, world_(world)
, linkNumber_(linkNumber)
{
    /**
     * Information on obstacle surface triangles.
     *
     * Saves all triangles that are feasible contact surfaces into member variable "trisFiltered".
     */

    for(uint k = 0; k < world_->terrains.size(); k++){
        Terrain* terrain_k = world_->terrains[k];
        const Geometry::CollisionMesh mesh = terrain_k->geometry->TriangleMeshCollisionData();

        for(uint j = 0; j < mesh.tris.size(); j++){
            Triangle3D tri;
            mesh.GetTriangle(j, tri);
            tris.push_back(tri);
        }
    }
}

Vector3 ContactConstraint_3D::getPos(const Eigen::Ref<const Eigen::VectorXd> &xd) const
{
    /**
     * Member function of class ContactConstraint_3D:
     * Returns position of given robot link in world coordinates.
     */

    Config q = cspace_->EigenVectorToConfig(xd);
    if (xd!=xd){
        std::cout << std::string (80, '-') << std::endl;
        std::cout << "EigenVector: " << std::endl;
        std::cout << xd << std::endl;
        std::cout << q << std::endl;
        exit(0);
    }


    robot_->UpdateConfig(q);
    robot_->UpdateGeometry();
    Vector3 zero;
    zero.setZero();

    Vector3 v;
    robot_->GetWorldPosition(zero, linkNumber_, v);

    return v;
}

void ContactConstraint_3D::function(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::VectorXd> out) const
{
    // ---------- Fixed Contact ------------------
    Vector3 contact = getPos(x);


    Vector3 closestPt = tris.at(0).closestPoint(contact);

    Real distances = 1000;
    //loop over all surface triangles to get triangle that's closest
    for (uint j = 0; j < tris.size(); j++) {
        Vector3 cP = tris.at(j).closestPoint(contact);
        Real d = contact.distance(cP);
        if (distances > d){

            distances = d;
            closestPt = cP;
        }
    }

    Real distVect = contact.distance(closestPt);

    out[0] = distVect;
}
