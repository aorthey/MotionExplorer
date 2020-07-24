#include <ompl/base/Constraint.h>
#include "planner/cspace/contact/ContactConstraint.h"
#include "planner/cspace/cspace_geometric_R2_CONTACT.h"


ContactConstraint::ContactConstraint
(GeometricCSpaceOMPLRCONTACT *cspace, int ambientSpaceDim, Robot *robot, RobotWorld *world, int linkNumber, std::string meshFrom):
ob::Constraint(ambientSpaceDim, 1)
        , cspace_(cspace)
        , robot_(robot)
        , world_(world)
        , linkNumber_(linkNumber)
        , meshFrom_(meshFrom)
{

    for(uint k = 0; k < world_->terrains.size(); k++){
        Terrain* terrain_k = world_->terrains[k];

        // adding only those Triangles to tris that belong to specified obstacle (meshFrom)
        if (terrain_k->name == meshFrom_){
            const Geometry::CollisionMesh mesh = terrain_k->geometry->TriangleMeshCollisionData();

            for(uint j = 0; j < mesh.tris.size(); j++){
                Triangle3D tri;
                mesh.GetTriangle(j, tri);

                Vector3 normal = tri.normal();
                double epsilon = 1e-10;

                // for 2D case, filter out all surface triangles with normal vector in z direction
                if(fabs((fabs(normal[2]) - 1.0))<epsilon){
                    //do nothing
                }
                else{
                    tris.push_back(tri);
                }
            }
        }
    }
}

Vector3 ContactConstraint::getPos(const Eigen::Ref<const Eigen::VectorXd> &xd) const
{
    /**
     * Member function of class ContactConstraint:
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
    Vector3 zero;
    zero.setZero();

    //NOTE: the world position is zero exactly at the point where link is
    //attached using a joint to the whole linkage. Check where your last fixed
    //joint is positioned, before questioning the validity of this method
    Vector3 v;
    robot_->GetWorldPosition(zero, linkNumber_, v);
    return v;
}

void ContactConstraint::function(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::VectorXd> out) const
{
    // ---------- Contact with chosen Link and chosen Obstacle ------------------
    Vector3 contact = getPos(x);
    Vector3 closestPt;
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