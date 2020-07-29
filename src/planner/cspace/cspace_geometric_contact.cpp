#include "planner/cspace/cspace_geometric_contact.h"
#include "planner/cspace/validitychecker/validity_checker_ompl.h"
#include "planner/cspace/contact/ConstraintIntersectionMultiMode.h"
#include "planner/cspace/contact/ConstraintContactFixed.h"
#include "planner/cspace/contact/ProjectedStateSpaceMultiMode.h"


GeometricCSpaceContact::GeometricCSpaceContact(RobotWorld *world_, int robot_idx):
    GeometricCSpaceOMPL(world_, robot_idx) 
{
}

ob::SpaceInformationPtr GeometricCSpaceContact::SpaceInformationPtr()
{
    if (!si) 
    {
        si = std::make_shared<ob::ConstrainedSpaceInformation>(SpacePtr());
        validity_checker = StateValidityCheckerPtr(si);
        si->setStateValidityChecker(validity_checker);
    }
    return si;
}

// void GeometricCSpaceContact::setConstraintsMode(TransitionMode mode)
// {
//   constraint_intersect->setMode(mode);
// }

void GeometricCSpaceContact::setGoalConstraints()
{
  constraint_intersect->setGoalMode();
  // setConstraintsMode(ACTIVE_CONSTRAINT_GOAL);
}

void GeometricCSpaceContact::addConstraintsToState(ob::State *state)
{
  ConstraintMode mode = constraint_intersect->getMode();
  state->as<ompl::base::ProjectedStateSpaceMultiMode::StateType>()->setMode(mode);
}

void GeometricCSpaceContact::setInitialConstraints()
{
  constraint_intersect->setInitMode();
  // setConstraintsMode(ACTIVE_CONSTRAINT_INITIAL);
}

std::vector<Triangle3D> GeometricCSpaceContact::getTrianglesOnMesh(std::string nameMesh)
{
    std::vector<Triangle3D> tris;
    for(uint k = 0; k < GetWorldPtr()->terrains.size(); k++)
    {
        Terrain* terrain_k = GetWorldPtr()->terrains[k];

        // adding only those Triangles to tris that belong to specified obstacle (meshFrom)
        if (terrain_k->name == nameMesh)
        {
            const Geometry::CollisionMesh mesh = terrain_k->geometry->TriangleMeshCollisionData();

            for(uint j = 0; j < mesh.tris.size(); j++)
            {
                Triangle3D tri;
                mesh.GetTriangle(j, tri);

                tris.push_back(tri);
            }
        }
    }
    return tris;
}

ob::ConstraintPtr GeometricCSpaceContact::getConstraints()
{
  return constraint_intersect;
}

void GeometricCSpaceContact::initConstraints(ob::StateSpacePtr Rn)
{
    std::cout << "Number of Links: " << robot->links.size() - 1 << std::endl;
    for(uint j = 0; j < input.contact_links.size(); j++)
    {
        ContactInformation cj = input.contact_links.at(j);

        int link = cj.robot_link_idx;
        if(cj.mode == "fixed")
        {
            std::cout << "Adding Fixed Contact Constraint:"
                      << " robot: " << cj.robot_name
                      << ", link: " << cj.robot_link << " (idx: " << cj.robot_link_idx << ")"
                      << " on mesh: " << cj.meshFrom << " (idx: " << cj.meshFromIdx << ")"
                      << std::endl;

            constraints.push_back
            (
                std::make_shared<ConstraintContactFixed>
                (
                  this, 
                  Rn->getDimension(), 
                  link, 
                  getTrianglesOnMesh(cj.meshFrom)
                )
            );

        // }else if(cj.mode == "transition")
        // {
        //     std::cout << "Adding Transition Contact Constraint:"
        //               << " robot: " << cj.robot_name
        //               << ", link: " << cj.robot_link << " (idx: " << cj.robot_link_idx << ")"
        //               << ", from mesh: " << cj.meshFrom << " (idx: " << cj.meshFromIdx << ")"
        //               << " to mesh: " << cj.meshTo << " (idx: " << cj.meshToIdx << ")"
        //               << std::endl;

        //     constraints.push_back(std::make_shared<TransitionConstraint2D>(this, Rn->getDimension(), robot, world, link, cj.meshFrom, cj.meshTo));

        }else
        {
            std::cout << "Could not identify contact mode" << std::endl;
            exit(0);
        }

    }

    constraint_intersect = std::make_shared<ConstraintIntersectionMultiMode>(
        Rn->getDimension(), constraints);
}
