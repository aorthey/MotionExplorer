#include <ompl/base/Constraint.h>
#include "planner/cspace/contact/ContactConstraint_3D.h"
#include "planner/cspace/cspace_geometric_R3_CONTACT.h"


ContactConstraint_3D::ContactConstraint_3D(GeometricCSpaceOMPLRCONTACT_3D *cspace, Robot *robot, RobotWorld *world):
ob::Constraint(5, 2)  // (x,y,z, theta at 1st link,phi at 2nd)
, cspace_(cspace)
, robot_(robot)
, world_(world)
{
    /**
     * Information on obstacle surface triangles.
     *
     * Saves all triangles that are feasible contact surfaces into member variable "trisFiltered".
     */

    std::vector<Triangle3D> tris;

    for(uint k = 0; k < world_->terrains.size(); k++){
        Terrain* terrain_k = world_->terrains[k];
        const Geometry::CollisionMesh mesh = terrain_k->geometry->TriangleMeshCollisionData();

        for(uint j = 0; j < mesh.tris.size(); j++){
            Triangle3D tri;
            mesh.GetTriangle(j, tri);
            tris.push_back(tri);
        }
    }
    for(uint l = 0; l < tris.size(); l++){
        Vector3 normal = tris.at(l).normal();
        double epsilon = 1e-10;

        if(fabs((fabs(normal[2]) - 1.0))<epsilon){
            //does nothing
        }
        else{
            // only x and y coordinates
            Vector2 a = Vector2(tris.at(l).a[0], tris.at(l).a[1]);
            Vector2 b = Vector2(tris.at(l).b[0], tris.at(l).b[1]);
            Vector2 c = Vector2(tris.at(l).c[0], tris.at(l).c[1]);

            cornerCoord.push_back(a);
            cornerCoord.push_back(b);
            cornerCoord.push_back(c);

            trisFiltered.push_back(tris.at(l));
        }
    }
    std::cout << "Environment has " << trisFiltered.size() << " triangles to make contact!" << std::endl;

}

Vector3 ContactConstraint_3D::getPos(const Eigen::Ref<const Eigen::VectorXd> &xd, int linkNumber) const
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
    robot_->GetWorldPosition(zero, linkNumber, v);

    return v;
}

void ContactConstraint_3D::function(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::VectorXd> out) const
{
    // ---------- Contact with First Link ------------------
    int firstLink = 6;
    Vector3 contact_firstLink = getPos(x, firstLink);

    Vector3 closestPt = trisFiltered.at(6).closestPoint(contact_firstLink);

    Real distances = 1000;
    //loop over all surface triangles to get triangle that's closest
    for (uint j = 0; j < trisFiltered.size(); j++) {
        Vector3 cP = trisFiltered.at(j).closestPoint(contact_firstLink);
        Real d = contact_firstLink.distance(cP);
        if (distances > d){

            distances = d;
            closestPt = cP;
        }
    }

    Real distVect = contact_firstLink.distance(closestPt);


//    // ---------- Contact with Last Link ------------------
//    int lastLink = robot_->links.size() - 1;
//    Vector3 contact_lastLink = getPos(x, lastLink);
//
//    Vector3 closestPt_last = trisFiltered.at(6).closestPoint(contact_lastLink);
//
//    Real distances_last = 1000;
//    //loop over all surface triangles to get triangle that's closest
//    for (uint j = 0; j < trisFiltered.size(); j++) {
//        Vector3 cP = trisFiltered.at(j).closestPoint(contact_lastLink);
//        Real d = contact_lastLink.distance(cP);
//        if (distances_last > d){
//
//            distances_last = d;
//            closestPt_last = cP;
//        }
//    }
//    Real distVect_last = contact_lastLink.distance(closestPt_last);

    out[0] = distVect;
    //out[0] = distVect_last;
}