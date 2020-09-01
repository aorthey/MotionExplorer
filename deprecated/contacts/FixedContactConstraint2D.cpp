#include <ompl/base/Constraint.h>
#include "planner/cspace/contact/FixedContactConstraint2D.h"
#include "planner/cspace/cspace_geometric_contact_2d.h"


FixedContactConstraint2D::FixedContactConstraint2D
(GeometricCSpaceContact2D *cspace, int ambientSpaceDim, Robot *robot, RobotWorld *world, int linkNumber, std::string meshFrom):
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

