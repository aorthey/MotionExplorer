#include "planner/cspace/cspace_geometric_Circular.h"
#include "planner/cspace/validitychecker/validity_checker_ompl.h"
#include <ompl/base/spaces/SO2StateSpace.h>
#include "common.h"
#include "gui/colors.h"
#include <KrisLibrary/GLdraw/drawextra.h>
#include <KrisLibrary/GLdraw/GLColor.h>

#include <ompl/util/Exception.h>

GeometricCSpaceOMPLCircular::GeometricCSpaceOMPLCircular(RobotWorld *world_, int robot_idx):
  BaseT(world_, robot_idx)
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
    return false;
}

void GeometricCSpaceOMPLCircular::DrawGL(GUIState& state)
{
  glDisable(GL_LIGHTING);
  glEnable(GL_BLEND);

  GLDraw::setColor(grey);
  glLineWidth(3);

  const double dstep = 0.01;
  glBegin(GL_LINE_LOOP);
  for(double d = 0; d < 2*M_PI; d+=dstep){
    Vector3 v = ProjectToVector3(d);
    GLDraw::glVertex3v(v);
  }
  glEnd();

  glDisable(GL_BLEND);
  glEnable(GL_LIGHTING);

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

    q[0] = radius_*cos(u);
    q[1] = radius_*sin(u);
    q[2] = zOffset_;
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
