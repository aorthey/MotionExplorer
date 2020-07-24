#include <ompl/base/Constraint.h>
#include "planner/cspace/contact/TransitionConstraint_3D.h"
#include "planner/cspace/cspace_geometric_R3_CONTACT.h"


TransitionConstraint_3D::TransitionConstraint_3D
        (GeometricCSpaceOMPLRCONTACT_3D *cspace, int ambientSpaceDim, Robot *robot, RobotWorld *world, uint linkNumber, std::string meshFrom, std::string meshTo):
        ContactConstraint_3D(cspace, ambientSpaceDim, robot, world, linkNumber, meshFrom)
{
    for(uint k = 0; k < world->terrains.size(); k++){
        Terrain* terrain_k = world->terrains[k];

        // adding only those Triangles to tris that belong to specified initial obstacle (meshFrom)
        if (terrain_k->name == meshFrom){
            const Geometry::CollisionMesh mesh = terrain_k->geometry->TriangleMeshCollisionData();

            for(uint j = 0; j < mesh.tris.size(); j++){
                Triangle3D tri;
                mesh.GetTriangle(j, tri);
                trisFrom.push_back(tri);
            }
        }

        // adding only those Triangles to tris that belong to specified goal obstacle (meshTo)
        if (terrain_k->name == meshTo){
            const Geometry::CollisionMesh mesh = terrain_k->geometry->TriangleMeshCollisionData();

            for(uint j = 0; j < mesh.tris.size(); j++){
                Triangle3D tri;
                mesh.GetTriangle(j, tri);
                trisTo.push_back(tri);
            }
        }
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
        // --------------- Mode 2: Contact with different contact surface ---------------
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
