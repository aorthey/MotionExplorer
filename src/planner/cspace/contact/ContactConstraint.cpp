#include <ompl/base/Constraint.h>
#include <Library/KrisLibrary/math3d/geometry3d.cpp>
#include "planner/cspace/contact/ContactConstraint.h"
#include "planner/cspace/cspace_geometric_R_CONTACT.h"


ContactConstraint::ContactConstraint(GeometricCSpaceOMPLRCONTACT *cspace, Robot *robot, RobotWorld *world):
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
            //std::cout << tris.at(l) << std::endl;
        }
    }
    std::cout << "Environment has " << trisFiltered.size() << " triangles to make contact!" << std::endl;

    // remove all duplicates of (2D) corner coordinates
    auto end = cornerCoord.end();
    for(auto it = cornerCoord.begin(); it != end; ++it){
        end = std::remove(it + 1, end, *it);
    }
    cornerCoord.erase(end, cornerCoord.end());
    //std::cout << "Filtered corner coordinates: " << cornerCoord << std::endl;

    // robot = world_->robots[robot_idx];

}

Vector3 ContactConstraint::getPos(const Eigen::Ref<const Eigen::VectorXd> &xd) const
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

    //int lastLink = robot_->links.size() - 1;
    int firstLink = 6; // first 6 links are virtual links x,y,z, yaw, pitch, roll

    //NOTE: the world position is zero exactly at the point where link is
    //attached using a joint to the whole linkage. Check where your last fixed
    //joint is positioned, before questioning the validity of this method
    Vector3 v;
    robot_->GetWorldPosition(zero, firstLink, v);

    return v;
}

void ContactConstraint::function(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::VectorXd> out) const
{

    Vector3 contact = getPos(x);
    //std::cout << "\nPosition of Link: " << contact << std::endl;

    Vector3 closestPt = trisFiltered.at(6).closestPoint(contact);

    Real distances = 100;
    //loop over all surface triangles to get triangle that's closest
    for (uint j = 0; j < trisFiltered.size(); j++) {
        Vector3 cP = trisFiltered.at(j).closestPoint(contact);
        Real d = contact.distance(cP);
        if (distances > d){

            distances = d;
            closestPt = cP;
            //std::cout << "Dist: " << distances << std::endl;
            //std::cout << "ClosestPt: " << cP << std::endl;

        }
    }

    //std::cout << "\nClosest Point on triangle: "<< closestPt << std::endl;

//    GeometricPrimitive3D gP3D = GeometricPrimitive3D(closestPt);
//    Real dist = gP3D.Distance(trisFiltered.at(0));
//     std::cout << "Check that 'closestPt' is on surface Triangle: " << dist << std::endl;

    Real distVect = contact.distance(closestPt);
    //std::cout << "\nDistance between First Link - Closest Point: " << distVect << std::endl;

    out[0] = distVect;
}