#include <ompl/base/Constraint.h>
#include "planner/cspace/contact/TransitionConstraint_3D.h"
#include "planner/cspace/cspace_geometric_R3_CONTACT.h"


TransitionConstraint_3D::TransitionConstraint_3D
(GeometricCSpaceOMPLRCONTACT_3D *cspace, int ambientSpaceDim, Robot *robot, RobotWorld *world, uint linkNumber):
ContactConstraint_3D(cspace, ambientSpaceDim, robot, world, linkNumber)
{

    std::vector<Triangle3D> tris;

    for(uint k = 0; k < world_->terrains.size(); k++){
        Terrain* terrain_k = world_->terrains[k];
        const Geometry::CollisionMesh mesh = terrain_k->geometry->TriangleMeshCollisionData();

        // writing all surface triangles into tris, as all are feasible in 3D case
        for(uint j = 0; j < mesh.tris.size(); j++){
            Triangle3D tri;
            mesh.GetTriangle(j, tri);
            tris.push_back(tri);
        }
    }

    for(uint l = 0; l < tris.size(); l++){
        // quick fix for distinction between two obstacles that dont touch, x coordinates positive or negative
        if(tris.at(l).a[0] < 0 && tris.at(l).a[0] > -2 && tris.at(l).b[0] > -2 && tris.at(l).c[0] > -2){
            trisFiltered_negative.push_back(tris.at(l));
            //std::cout << "Triangle with negative x coord: " << tris.at(l) << std::endl;

        }else if (tris.at(l).a[0] > 0 && tris.at(l).a[0] < 2.5 && tris.at(l).b[0] < 2.5 && tris.at(l).c[0] < 2.5){
            trisFiltered.push_back(tris.at(l));
            //std::cout << "Triangle with positive x coord: " << tris.at(l) << std::endl;
        }

//            // quick fix for distinction between two obstacles that dont touch, Y coordinates positive or negative
//            if (tris.at(l).a[0] < 2.5 && tris.at(l).b[0] < 2.5 && tris.at(l).c[0] < 2.5){
//                if(tris.at(l).a[1] > 0){
//                    trisFiltered_negative.push_back(tris.at(l));
//                } else{
//                    trisFiltered.push_back(tris.at(l));
//                }
//            }

    }

}

int TransitionConstraint_3D::getMode()
{
  return mode;
}

void TransitionConstraint_3D::setMode(int newMode)
{
    mode = static_cast<Mode>(newMode);
}

void TransitionConstraint_3D::function(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::VectorXd> out) const
{
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
