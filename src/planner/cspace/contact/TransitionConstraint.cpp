#include <ompl/base/Constraint.h>
#include "planner/cspace/contact/TransitionConstraint.h"
#include "planner/cspace/cspace_geometric_R2_CONTACT.h"


TransitionConstraint::TransitionConstraint
(GeometricCSpaceOMPLRCONTACT *cspace, int ambientSpaceDim, Robot *robot, RobotWorld *world, uint linkNumber, uint obstacleNumber):
ContactConstraint(cspace, ambientSpaceDim, robot, world, linkNumber, obstacleNumber)
{
    /**
     * Information on obstacle surface triangles.
     *
     * Saves all triangles that are feasible contact surfaces into member variable "trisFiltered".
     */

    std::vector<Triangle3D> tris;

    for(uint k = 0; k < world->terrains.size(); k++){
        Terrain* terrain_k = world->terrains[k];
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

//            // quick fix for distinction between two obstacles that dont touch, x coordinates positive or negative
//            if(tris.at(l).a[0] < 0 && tris.at(l).a[0] > -2 && tris.at(l).b[0] > -2 && tris.at(l).c[0] > -2){
//                trisFiltered_negative.push_back(tris.at(l));
//                //std::cout << "Triangle with negative x coord: " << tris.at(l) << std::endl;
//
//            }else if (tris.at(l).a[0] > 0 && tris.at(l).a[0] < 2.5 && tris.at(l).b[0] < 2.5 && tris.at(l).c[0] < 2.5){
//                trisFiltered.push_back(tris.at(l));
//                //std::cout << "Triangle with positive x coord: " << tris.at(l) << std::endl;
//            }

            // quick fix for distinction between two obstacles that dont touch, Y coordinates positive or negative
            if (tris.at(l).a[0] < 2.5 && tris.at(l).b[0] < 2.5 && tris.at(l).c[0] < 2.5){
                if(tris.at(l).a[1] > 0){
                    trisFiltered_negative.push_back(tris.at(l));
                    std::cout << "Y-Coord greater than 0: " << tris.at(l) << std::endl;
                } else{
                    trisFiltered.push_back(tris.at(l));
                    std::cout << "Y-Coord smaller than 0: " << tris.at(l) << std::endl;
                }
            }

        }
    }

}

int TransitionConstraint::getMode()
{
  return mode;
}

void TransitionConstraint::setMode(int newMode)
{
    mode = static_cast<Mode>(newMode);
}

void TransitionConstraint::function(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::VectorXd> out) const
{
//// remember to #include <Library/KrisLibrary/math3d/geometry3d.cpp> for this to work
////    GeometricPrimitive3D gP3D = GeometricPrimitive3D(closestPt);
////    Real dist = gP3D.Distance(trisFiltered.at(0));
////     std::cout << "Check that 'closestPt' is on surface Triangle: " << dist << std::endl;


    // Variable "mode" defines which constraint function is active
    // -> Contact with initial surface
    // OR Free from constraint
    // OR Contact with different surface

    if (mode == ACTIVE_CONSTRAINT_INITIAL){
        // --------------- Mode 0: Contact with initial contact surface ---------------
        Vector3 contact = getPos(x);

        Vector3 closestPt;

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

    } else if (mode == NO_ACTIVE_CONSTRAINT){
        // --------------- Mode 1: No contact constraint ---------------
        out[0] = 0.0;

    } else if (mode == ACTIVE_CONSTRAINT_GOAL){
        // --------------- Mode 2: Contact with different contact surface ---------------
        Vector3 contact = getPos(x);

        Vector3 closestPt;  // !!! SURFACE CHOICE HERE !!!

        Real distances = 1000;
        //loop over all surface triangles to get triangle that's closest
        for (uint j = 0; j < trisFiltered_negative.size(); j++) {
            Vector3 cP = trisFiltered_negative.at(j).closestPoint(contact);
            Real d = contact.distance(cP);

            if (distances > d){
                distances = d;
                closestPt = cP;
            }
        }
        Real distVect = contact.distance(closestPt);

        out[0] = distVect;

    }else
    {
      OMPL_ERROR("Mode %d not recognized", mode);
      throw ompl::Exception("UNKNOWN MODE");
    }
}
