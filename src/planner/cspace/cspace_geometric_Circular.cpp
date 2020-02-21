#include "planner/cspace/cspace_geometric_Circular.h"
#include "planner/cspace/validitychecker/validity_checker_ompl.h"
#include <ompl/base/spaces/SO2StateSpace.h>
#include "common.h"

#include <ompl/util/Exception.h>

GeometricCSpaceOMPLCircular::GeometricCSpaceOMPLCircular(RobotWorld *world_, int robot_idx):
  GeometricCSpaceOMPLMobius(world_, robot_idx)
{
}

void GeometricCSpaceOMPLCircular::initSpace()
{
    ob::StateSpacePtr SO2(std::make_shared<ob::SO2StateSpace>());
    space = SO2;
}

void GeometricCSpaceOMPLCircular::print(std::ostream& out) const
{
    out << "Circular Space";
}

bool GeometricCSpaceOMPLCircular::IsPlanar(){
    return true;
}

void GeometricCSpaceOMPLCircular::ConfigToOMPLState(const Config &q, ob::State *qompl)
{
    double x = q[0];
    double y = q[1];

    double r = sqrt(x*x + y*y);
    double u = atan2(y/r, x/r);

    ob::SO2StateSpace::StateType *qomplSO2 = 
      qompl->as<ob::SO2StateSpace::StateType>();

    qomplSO2->value = u;
    space->enforceBounds(qomplSO2);
}

double GeometricCSpaceOMPLCircular::OMPLStateToSO2Value(const ob::State *qompl)
{
    const ob::SO2StateSpace::StateType *qomplSO2 = 
      qompl->as<ob::SO2StateSpace::StateType>();

    return qomplSO2->value;
}


Vector3 GeometricCSpaceOMPLCircular::ProjectToVector3(double u)
{
    Config q = ProjectToConfig(u);
    double x = q[0];
    double y = q[1];
    double z = q[2];
    Vector3 vector(x,y,z);
    return vector;
}

Config GeometricCSpaceOMPLCircular::ProjectToConfig(double u)
{
    Config q;q.resize(robot->q.size());q.setZero();

    double R = 1;
    q[0] = R*cos(u);
    q[1] = R*sin(u);
    q[2] = 0;
    return q;
}

Config GeometricCSpaceOMPLCircular::OMPLStateToConfig(const ob::State *x)
{
    double u = OMPLStateToSO2Value(x);
    return ProjectToConfig(u);
}

Vector3 GeometricCSpaceOMPLCircular::getXYZ(const ob::State *x)
{
    double u = OMPLStateToSO2Value(x);
    return ProjectToVector3(u);
}
