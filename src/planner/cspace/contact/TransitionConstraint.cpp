#include <ompl/base/Constraint.h>
#include "planner/cspace/contact/TransitionConstraint.h"
#include "planner/cspace/cspace_geometric_R2_CONTACT.h"


TransitionConstraint::TransitionConstraint
(GeometricCSpaceOMPLRCONTACT *cspace, Robot *robot, RobotWorld *world, uint linkNumber, uint mode):
ob::Constraint(5, 2)  // (x,y,z, theta at 1st link,phi at 2nd) !!! gotta check these numbers !!!
        , cspace_(cspace)
        , robot_(robot)
        , world_(world)
        , linkNumber_(linkNumber)
        , mode_(mode)
{
    /**
     * Information on obstacle surface triangles.
     *
     * Saves all triangles that are feasible contact surfaces into member variable "trisFiltered".
     */

    std::vector<Triangle3D> tris;

    for(uint k = 0; k < world_->terrains.size(); k++){
        Terrain* terrain_k = world_->terrains[k];
        const Geometry::CollisionMesh mesh = terrain_k->geometry->TriangleMeshCollisionData();//different mesh

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
            //do nothing
        }
        else{

            trisFiltered.push_back(tris.at(l));
        }
    }

}

Vector3 TransitionConstraint::getPos(const Eigen::Ref<const Eigen::VectorXd> &xd) const
{
    /**
     * Member function of class ContactConstraint:
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

    //NOTE: the world position is zero exactly at the point where link is
    //attached using a joint to the whole linkage. Check where your last fixed
    //joint is positioned, before questioning the validity of this method
    Vector3 v;
    robot_->GetWorldPosition(zero, linkNumber_, v);

    return v;
}

void TransitionConstraint::function(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::VectorXd> out) const
{
    // ---------- Contact with First Link ------------------
//    int firstLink = 6;
//    Vector3 contact_firstLink = getPos(x, firstLink);
//
//    Vector3 closestPt = trisFiltered.at(6).closestPoint(contact_firstLink);
//
//    Real distances = 1000;
//    //loop over all surface triangles to get triangle that's closest
//    for (uint j = 0; j < trisFiltered.size(); j++) {
//        Vector3 cP = trisFiltered.at(j).closestPoint(contact_firstLink);
//        Real d = contact_firstLink.distance(cP);
//        if (distances > d){
//
//            distances = d;
//            closestPt = cP;
//        }
//    }
//// remember to #include <Library/KrisLibrary/math3d/geometry3d.cpp> for this to work
////    GeometricPrimitive3D gP3D = GeometricPrimitive3D(closestPt);
////    Real dist = gP3D.Distance(trisFiltered.at(0));
////     std::cout << "Check that 'closestPt' is on surface Triangle: " << dist << std::endl;
//
//    Real distVect = contact_firstLink.distance(closestPt);


    //out[0] = distVect;

    // Variable "mode" defines which constraint function is active
    // -> Contact with initial surface
    // OR Free from constraint
    // OR Contact with different surface

    if (mode_ == 0){
        // --------------- Mode 0: Contact with initial contact surface ---------------
        Vector3 contact = getPos(x);

        Vector3 closestPt = trisFiltered.at(6).closestPoint(contact);

        Real distances = 1000;
        //loop over all surface triangles to get triangle that's closest
        for (uint j = 0; j < trisFiltered.size(); j++) {
            Vector3 cP = trisFiltered.at(j).closestPoint(contact);
            Real d = contact.distance(cP);

            if (distances > d){
                distances = d;
                closestPt = cP;
            }
        }
        Real distVect = contact.distance(closestPt);

        out[0] = distVect;

    } else if (mode_ == 1){
        // --------------- Mode 1: No contact constraint ---------------
        out[0] = 0;

    } else if (mode_ == 2){
        // --------------- Mode 2: Contact with different contact surface ---------------
        Vector3 contact = getPos(x);

        Vector3 closestPt = trisFiltered.at(6).closestPoint(contact);  // !!! SURFACE CHOICE HERE !!!

        Real distances = 1000;
        //loop over all surface triangles to get triangle that's closest
        for (uint j = 0; j < trisFiltered.size(); j++) {
            Vector3 cP = trisFiltered.at(j).closestPoint(contact);
            Real d = contact.distance(cP);

            if (distances > d){
                distances = d;
                closestPt = cP;
            }
        }
        Real distVect = contact.distance(closestPt);

        out[0] = distVect;

    }

}
