#include "planner/cspace/cspace_geometric_Mobius.h"
#include "planner/cspace/validitychecker/validity_checker_ompl.h"
#include "ompl/base/spaces/SO2StateSpaceFullInterpolate.h"
#include "common.h"

#include <ompl/util/Exception.h>

GeometricCSpaceOMPLMobius::GeometricCSpaceOMPLMobius(RobotWorld *world_, int robot_idx):
  GeometricCSpaceOMPL(world_, robot_idx)
{
}

void GeometricCSpaceOMPLMobius::initSpace()
{
    //###########################################################################
    // Create bundle space, locally homeomorphic to SO(2) x R^1
    //###########################################################################

    ob::StateSpacePtr SO2(std::make_shared<ob::SO2StateSpaceFullInterpolate>());
    ob::StateSpacePtr R1(std::make_shared<ob::RealVectorStateSpace>(1));

    space = SO2 + R1;

    space->as<ob::CompoundStateSpace>()->as<ob::RealVectorStateSpace>(1)->setBounds(-0.5, +0.5);
}

void GeometricCSpaceOMPLMobius::print(std::ostream& out) const
{
    out << "Mobius Strip Space";
}

bool GeometricCSpaceOMPLMobius::IsPlanar(){
    return false;
}

void GeometricCSpaceOMPLMobius::ConfigToOMPLState(const Config &q, ob::State *qompl)
{
    double x = q[0];
    double y = q[1];
    double z = q[2];

    double r = sqrt(x*x + y*y);
    double u = atan2(y/r, x/r);

    double v = 0;
    if(u > 0.01 || u < -0.01)
        v = z / (sin(0.5*u));
    else 
        v = (x/cos(u) - 1)/cos(0.5*u);

    ob::SO2StateSpaceFullInterpolate::StateType *qomplSO2 = 
      qompl->as<ob::CompoundState>()->as<ob::SO2StateSpaceFullInterpolate::StateType>(0);
    ob::RealVectorStateSpace::StateType *qomplRnSpace = 
      qompl->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(1);

    qomplSO2->value = u;
    qomplRnSpace->values[0] = v;
}

Vector3 GeometricCSpaceOMPLMobius::ProjectToVector3(double u, double v)
{
    Config q = ProjectToConfig(u, v);
    double x = q[0];
    double y = q[1];
    double z = q[2];
    Vector3 vector(x,y,z);
    return vector;
}

Config GeometricCSpaceOMPLMobius::ProjectToConfig(double u, double v)
{
    Config q;q.resize(robot->q.size());q.setZero();

    double R = 1 + v*cos(0.5*u);
    q[0] = R*cos(u);
    q[1] = R*sin(u);
    q[2] = v*sin(0.5*u);
    return q;
}


double GeometricCSpaceOMPLMobius::OMPLStateToSO2Value(const ob::State *qompl)
{
    const ob::SO2StateSpaceFullInterpolate::StateType *qomplSO2 = 
      qompl->as<ob::CompoundState>()->as<ob::SO2StateSpaceFullInterpolate::StateType>(0);

    return qomplSO2->value;
}

double GeometricCSpaceOMPLMobius::OMPLStateToRValue(const ob::State *qompl)
{
    const ob::RealVectorStateSpace::StateType *qomplRnSpace = 
      qompl->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(1);

    return qomplRnSpace->values[0];
}


Config GeometricCSpaceOMPLMobius::OMPLStateToConfig(const ob::State *x)
{
    double u = OMPLStateToSO2Value(x);
    double v = OMPLStateToRValue(x);
    return ProjectToConfig(u, v);
}

Vector3 GeometricCSpaceOMPLMobius::getXYZ(const ob::State *x)
{
    double u = OMPLStateToSO2Value(x);
    double v = OMPLStateToRValue(x);
    return ProjectToVector3(u, v);
}
