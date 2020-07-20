#include <ompl/base/Constraint.h>
#include "planner/cspace/contact/ContactConstraint.h"
#include "planner/cspace/cspace_geometric_R2_CONTACT.h"


ContactConstraint::ContactConstraint(GeometricCSpaceOMPLRCONTACT *cspace, Robot *robot, RobotWorld *world, uint linkNumber, uint obstacleNumber):
        ob::Constraint(5, 1)  // (x,y,z, theta at 1st link,phi at 2nd)
        , cspace_(cspace)
        , robot_(robot)
        , world_(world)
        , linkNumber_(linkNumber)
        , obstacleNumber_(obstacleNumber)
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
//
//            // quick fix for distinction between two obstacles that dont touch, x coordinates positive or negative
//            if(tris.at(l).a[0] < 0 && tris.at(l).a[0] > -2 && tris.at(l).b[0] > -2 && tris.at(l).c[0] > -2){
//                trisFiltered_negative.push_back(tris.at(l));
//
//            }else if (tris.at(l).a[0] > 0 && tris.at(l).a[0] < 2.5 && tris.at(l).b[0] < 2.5 && tris.at(l).c[0] < 2.5){
//                trisFiltered.push_back(tris.at(l));
//
//            }

            // quick fix for distinction between two obstacles that dont touch, Y coordinates positive or negative
            if(tris.at(l).a[0] > -2 && tris.at(l).b[0] > -2 && tris.at(l).c[0] > -2){
                if(tris.at(l).a[1] > 0){
                    trisFiltered_negative.push_back(tris.at(l));

                } else{
                    trisFiltered.push_back(tris.at(l));
                }
            }

        }
    }

    // remove all duplicates of (2D) corner coordinates
    auto end = cornerCoord.end();
    for(auto it = cornerCoord.begin(); it != end; ++it){
        end = std::remove(it + 1, end, *it);
    }
    cornerCoord.erase(end, cornerCoord.end());

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

    if(obstacleNumber_ == 0){
        // obstacle 0 is initial surface -> contact starts here

        //loop over all surface triangles to get triangle that's closest
        for (uint j = 0; j < trisFiltered.size(); j++) {
            Vector3 cP = trisFiltered.at(j).closestPoint(contact);
            Real d = contact.distance(cP);

            if (distances > d){
                distances = d;
                closestPt = cP;
            }
        }

    }else if(obstacleNumber_ == 1){
        // obstacle 1 has only negative x cordinates

        //loop over all surface triangles to get triangle that's closest
        for (uint j = 0; j < trisFiltered_negative.size(); j++) {
            Vector3 cP = trisFiltered_negative.at(j).closestPoint(contact);
            Real d = contact.distance(cP);

            if (distances > d){
                distances = d;
                closestPt = cP;
            }
        }
    }else{
        std::cout << "Invalid Obstacle "  << obstacleNumber_ << std::endl;
        std::exit(0);
    }



    Real distVect = contact.distance(closestPt);

    out[0] = distVect;
}