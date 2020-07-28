#include "planner/cspace/cspace_geometric_contact.h"
#include "planner/cspace/validitychecker/validity_checker_ompl.h"
#include "planner/cspace/contact/TransitionConstraint2D.h"
#include "planner/cspace/contact/TransitionConstraint3D.h"


GeometricCSpaceContact::GeometricCSpaceContact(RobotWorld *world_, int robot_idx):
    GeometricCSpaceOMPL(world_, robot_idx) 
{
}

ob::SpaceInformationPtr GeometricCSpaceContact::SpaceInformationPtr()
{
    if (!si) {
        si = std::make_shared<ob::ConstrainedSpaceInformation>(SpacePtr());
        validity_checker = StateValidityCheckerPtr(si);
        si->setStateValidityChecker(validity_checker);
    }
    return si;
}

void GeometricCSpaceContact::setConstraintsMode(TransitionMode mode)
{
    for(uint i = 0; i < constraints.size(); i++)
    {
        ob::ConstraintPtr cPi = constraints.at(i);

        auto tCP = std::dynamic_pointer_cast<TransitionConstraint2D>(cPi);
        auto tCP3D = std::dynamic_pointer_cast<TransitionConstraint3D>(cPi);
        if (tCP != nullptr)
        {
          tCP->setMode(mode);
        }
        if (tCP3D != nullptr)
        {
          tCP3D->setMode(mode);
        }
    }

}
void GeometricCSpaceContact::setGoalConstraints()
{
  setConstraintsMode(ACTIVE_CONSTRAINT_GOAL);
}
void GeometricCSpaceContact::setInitialConstraints()
{
  setConstraintsMode(ACTIVE_CONSTRAINT_INITIAL);
}
