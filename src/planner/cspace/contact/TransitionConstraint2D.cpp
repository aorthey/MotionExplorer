#include <ompl/base/Constraint.h>
#include "planner/cspace/contact/TransitionConstraint2D.h"
#include "planner/cspace/cspace_geometric_contact_2d.h"


TransitionConstraint2D::TransitionConstraint2D
(GeometricCSpaceContact2D *cspace, int ambientSpaceDim, Robot *robot, RobotWorld *world, uint linkNumber, std::string meshFrom, std::string meshTo):
ContactConstraint2D(cspace, ambientSpaceDim, robot, world, linkNumber, meshFrom)
{

    for(uint k = 0; k < world->terrains.size(); k++){
        Terrain* terrain_k = world->terrains[k];

        // adding only those Triangles to tris that belong to specified obstacle (meshFrom)
        if (terrain_k->name == meshFrom){
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
                    trisFrom.push_back(tri);
                }
            }
        }

        // adding only those Triangles to tris that belong to specified obstacle (meshTo)
        if (terrain_k->name == meshTo){
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
                    trisTo.push_back(tri);
                }

            }
        }
    }
}

int TransitionConstraint2D::getMode()
{
  return mode;
}

void TransitionConstraint2D::setMode(int newMode)
{
    mode = static_cast<TransitionMode>(newMode);
}

void TransitionConstraint2D::function(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::VectorXd> out) const
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
        for (uint j = 0; j < trisFrom.size(); j++) {
            Vector3 cP = trisFrom.at(j).closestPoint(contact);
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
        // --------------- Mode 2: Contact with goal contact surface ---------------
        Vector3 contact = getPos(x);
        Vector3 closestPt;
        Real distances = 1000;

        //loop over all surface triangles to get triangle that's closest
        for (uint j = 0; j < trisTo.size(); j++) {
            Vector3 cP = trisTo.at(j).closestPoint(contact);
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
