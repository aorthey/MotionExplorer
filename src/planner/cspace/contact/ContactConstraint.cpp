#include <ompl/base/Constraint.h>
#include <Library/KrisLibrary/math3d/geometry3d.cpp>
#include "planner/cspace/contact/ContactConstraint.h"
#include "planner/cspace/cspace_geometric_R_CONTACT.h"


ContactConstraint::ContactConstraint(Robot *robot, RobotWorld *world, int robot_idx):
ob::Constraint(5, 2)  // (x,y,z, theta at 1st link,phi at 2nd)
, robot_(robot)
, world_(world)
,robot_idx_(robot_idx)
{
    /**
     * Information on obstacle surface triangles.
     *
     * Filtering list of all triangles such that feasible contact surfaces remain.
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

Vector3 ContactConstraint::getPos(const Eigen::VectorXd xd) const
{
    /**
     * Member function of class ContactConstraint:
     * Calculates position of given robot link in world coordinates.
     */
    std::cout << "\nEigenVector: \n" << xd << std::endl;

    GeometricCSpaceOMPLRCONTACT geometricCSpaceOmplRContact = GeometricCSpaceOMPLRCONTACT(world_, robot_idx_);

    Config q = geometricCSpaceOmplRContact.EigenVectorToConfig(xd);

    robot_->UpdateConfig(q);
    std::cout << "Config q: " << q << std::endl;
    robot_->UpdateGeometry();
    Vector3 qq;
    Vector3 zero;
    zero.setZero();
    //int lastLink = robot_->links.size() - 1;
    int firstLink = 0; // maybe .type to check type of last/first link, ball endeffector

    //NOTE: the world position is zero exactly at the point where link is
    //attached using a joint to the whole linkage. Check where your last fixed
    //joint is positioned, before questioning the validity of this method
    robot_->GetWorldPosition(zero, firstLink, qq);

    double x = qq[0];
    double y = q[1];
    double z = qq[2];
    Vector3 v(x,y,z);

    std::cout << "Vector xyz: " << v << std::endl;

    return v;
}

void ContactConstraint::function(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::VectorXd> out) const
{

    Vector3 contact = getPos(x);
    std::cout << "\nPosition of First Link: " << contact << std::endl;

    Vector3 closestPt = trisFiltered.at(6).closestPoint(contact);
    //std::cout << "\nFiltered Surface Triangles: " << trisFiltered.at(6) << std::endl;
    std::cout << "\nClosest Point on triangle: "<< closestPt << std::endl;

    //GeometricPrimitive3D gP3D = GeometricPrimitive3D(closestPt);
    //Real dist = gP3D.Distance(trisFiltered.at(0));
    // std::cout << "Check that 'closestPt' is on surface Triangle: " << dist << std::endl;

    Real distVect = contact.distance(closestPt);
    std::cout << "\nDistance between First Link - Closest Point: " << distVect << std::endl;

    //std::exit(1);
    out[0] = distVect;
}
