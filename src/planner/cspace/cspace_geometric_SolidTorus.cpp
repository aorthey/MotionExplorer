#include "planner/cspace/cspace_geometric_SolidTorus.h"
#include "planner/cspace/validitychecker/validity_checker_ompl.h"
#include <ompl/base/spaces/SO2StateSpace.h>
#include "common.h"
#include "gui/colors.h"
#include <KrisLibrary/GLdraw/drawextra.h>
#include <KrisLibrary/GLdraw/GLColor.h>

#include <ompl/util/Exception.h>

GeometricCSpaceOMPLSolidTorus::GeometricCSpaceOMPLSolidTorus(RobotWorld *world_, int robot_idx):
  GeometricCSpaceOMPL(world_, robot_idx)
{
}

void GeometricCSpaceOMPLSolidTorus::initSpace()
{
    ob::StateSpacePtr SO2(std::make_shared<ob::SO2StateSpace>());
    ob::StateSpacePtr R2(std::make_shared<ob::RealVectorStateSpace>(2));

    double radius = 0.5*(radiusOuter_ - radiusInner_);

    vector<double> low;
    low.push_back(radiusInner_);
    low.push_back(-radius);
    vector<double> high;
    high.push_back(radiusOuter_);
    high.push_back(+radius);

    ob::RealVectorBounds bounds(2);
    bounds.low = low;
    bounds.high = high;
    bounds.check();

    R2->as<ob::RealVectorStateSpace>()->setBounds(bounds);

    space = SO2 + R2;
}

void GeometricCSpaceOMPLSolidTorus::print(std::ostream& out) const
{
    out << "SolidTorus Space";
}

bool GeometricCSpaceOMPLSolidTorus::IsPlanar(){
    return false;
}

void GeometricCSpaceOMPLSolidTorus::DrawGL(GUIState& state)
{
  glDisable(GL_LIGHTING);
  glEnable(GL_BLEND);

  GLDraw::setColor(black);
  glLineWidth(3);

  const double dstep = 0.01;
  glBegin(GL_LINE_LOOP);

  for(double d = 0; d < 2*M_PI; d+=dstep){
    Vector3 v = ProjectToVector3(d, radiusInner_);
    GLDraw::glVertex3v(v);
  }
  for(double d = 0; d < 2*M_PI; d+=dstep){
    Vector3 v = ProjectToVector3(d, radiusOuter_);
    GLDraw::glVertex3v(v);
  }
  glEnd();


  glDisable(GL_BLEND);
  glEnable(GL_LIGHTING);

}

void GeometricCSpaceOMPLSolidTorus::ConfigToOMPLState(const Config &q, ob::State *qompl)
{
    double x = q[0];
    double y = q[1];
    // double z = q[2] - zOffset_;

    double r = sqrt(x*x + y*y);
    double u = atan2(y/r, x/r);

    ob::SO2StateSpace::StateType *qomplSO2 = 
      qompl->as<ob::CompoundState>()->as<ob::SO2StateSpace::StateType>(0);
    ob::RealVectorStateSpace::StateType *qomplRnSpace = 
      qompl->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(1);

    qomplSO2->value = u;
    qomplRnSpace->values[0] = r;
    qomplRnSpace->values[1] = r;
}

Vector3 GeometricCSpaceOMPLSolidTorus::ProjectToVector3(double u, double r)
{
    Config q = ProjectToConfig(u, r);
    double x = q[0];
    double y = q[1];
    double z = q[2];
    Vector3 vector(x,y,z);
    return vector;
}

Config GeometricCSpaceOMPLSolidTorus::ProjectToConfig(double u, double r)
{
    Config q;q.resize(robot->q.size());q.setZero();

    q[0] = r*cos(u);
    q[1] = r*sin(u);
    q[2] = zOffset_;
    return q;
}

double GeometricCSpaceOMPLSolidTorus::OMPLStateToSO2Value(const ob::State *qompl)
{
    const ob::SO2StateSpace::StateType *qomplSO2 = 
      qompl->as<ob::CompoundState>()->as<ob::SO2StateSpace::StateType>(0);

    return qomplSO2->value;
}

double GeometricCSpaceOMPLSolidTorus::OMPLStateToRValue(const ob::State *qompl)
{
    const ob::RealVectorStateSpace::StateType *qomplRnSpace = 
      qompl->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(1);

    return qomplRnSpace->values[0];
}

Config GeometricCSpaceOMPLSolidTorus::OMPLStateToConfig(const ob::State *x)
{
    double u = OMPLStateToSO2Value(x);
    double r = OMPLStateToRValue(x);
    return ProjectToConfig(u, r);
}

Vector3 GeometricCSpaceOMPLSolidTorus::getXYZ(const ob::State *x)
{
    double u = OMPLStateToSO2Value(x);
    double r = OMPLStateToRValue(x);
    return ProjectToVector3(u, r);
}
