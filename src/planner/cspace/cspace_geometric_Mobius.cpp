#include "planner/cspace/cspace_geometric_Mobius.h"
#include "planner/cspace/validitychecker/validity_checker_ompl.h"
// #include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/spaces/MobiusStateSpace.h>
#include "common.h"
#include "gui/colors.h"
#include <KrisLibrary/GLdraw/drawextra.h>
#include <KrisLibrary/GLdraw/GLColor.h>

#include <ompl/util/Exception.h>

GeometricCSpaceOMPLMobius::GeometricCSpaceOMPLMobius(RobotWorld *world_, int robot_idx):
  GeometricCSpaceOMPL(world_, robot_idx)
{
}

void GeometricCSpaceOMPLMobius::initSpace()
{
    // ob::StateSpacePtr SO2(std::make_shared<ob::SO2StateSpace>());
    // ob::StateSpacePtr R1(std::make_shared<ob::RealVectorStateSpace>(1));

    // R1->as<ob::RealVectorStateSpace>()->setBounds(-intervalMax, +intervalMax);

    // space = SO2 + R1;

    ob::StateSpacePtr M(std::make_shared<ob::MobiusStateSpace>(intervalMax));
		space = M;
}

void GeometricCSpaceOMPLMobius::print(std::ostream& out) const
{
    out << "Mobius Strip Space";
}

bool GeometricCSpaceOMPLMobius::IsPlanar(){
    return false;
}

void GeometricCSpaceOMPLMobius::DrawGL(GUIState& state)
{
  glDisable(GL_LIGHTING);
  glEnable(GL_BLEND);

  GLDraw::setColor(black);
  glLineWidth(3);

  const double dstep = 0.01;
  glBegin(GL_LINE_LOOP);
  for(double d = 0; d < 2*M_PI; d+=dstep){
    Vector3 v = ProjectToVector3(d, -intervalMax);
    // Vector3 v2 = ProjectToVector3(d, +intervalMax);
    GLDraw::glVertex3v(v);
  }
  for(double d = 0; d < 2*M_PI; d+=dstep){
    Vector3 v = ProjectToVector3(d, +intervalMax);
    GLDraw::glVertex3v(v);
  }
  glEnd();

  GLDraw::setColor(grey);
  glLineWidth(1);

  glBegin(GL_LINES);
  const double dstep2 = 0.1;
  for(double d = 0; d < 2*M_PI; d+=dstep2){
    Vector3 v1 = ProjectToVector3(d, -intervalMax);
    Vector3 v2 = ProjectToVector3(d, +intervalMax);
    GLDraw::glVertex3v(v1);
    GLDraw::glVertex3v(v2);
  }
  glEnd();

  glDisable(GL_BLEND);
  glEnable(GL_LIGHTING);

}

void GeometricCSpaceOMPLMobius::ConfigToOMPLState(const Config &q, ob::State *qompl)
{
    double x = q[0];
    double y = q[1];
    double z = q[2] - zOffset_;

    double r = sqrt(x*x + y*y);
    double u = atan2(y/r, x/r);

    double v = 0;
    if(u > 0.01 || u < -0.01)
        v = z / (sin(0.5*u));
    else 
        v = (x/cos(u) - radius_)/cos(0.5*u);

    ob::SO2StateSpace::StateType *qomplSO2 = 
      qompl->as<ob::CompoundState>()->as<ob::SO2StateSpace::StateType>(0);
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

    double R = radius_ + v*cos(0.5*u);
    q[0] = R*cos(u);
    q[1] = R*sin(u);
    q[2] = v*sin(0.5*u) + zOffset_;
    return q;
}

double GeometricCSpaceOMPLMobius::OMPLStateToSO2Value(const ob::State *qompl)
{
    const ob::SO2StateSpace::StateType *qomplSO2 = 
      qompl->as<ob::CompoundState>()->as<ob::SO2StateSpace::StateType>(0);

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
