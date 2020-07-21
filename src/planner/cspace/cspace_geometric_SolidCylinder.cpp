#include "planner/cspace/cspace_geometric_SolidCylinder.h"
#include "planner/cspace/validitychecker/validity_checker_ompl.h"
#include <ompl/base/spaces/SO2StateSpace.h>
#include "common.h"
#include "gui/colors.h"
#include <KrisLibrary/GLdraw/drawextra.h>
#include <KrisLibrary/GLdraw/GLColor.h>

#include <boost/math/constants/constants.hpp>

using namespace boost::math::double_constants;

#include <ompl/util/Exception.h>

GeometricCSpaceOMPLSolidCylinder::GeometricCSpaceOMPLSolidCylinder(RobotWorld *world_, int robot_idx):
  GeometricCSpaceOMPL(world_, robot_idx)
{
}

void GeometricCSpaceOMPLSolidCylinder::initSpace()
{
    ob::StateSpacePtr SO2(std::make_shared<ob::SO2StateSpace>());
    ob::StateSpacePtr R2(std::make_shared<ob::RealVectorStateSpace>(2));

    vector<double> low;
    low.push_back(radiusInner_);
    low.push_back(0);
    vector<double> high;
    high.push_back(radiusOuter_);
    high.push_back(height_);

    ob::RealVectorBounds bounds(2);
    bounds.low = low;
    bounds.high = high;
    bounds.check();

    R2->as<ob::RealVectorStateSpace>()->setBounds(bounds);

    space = SO2 + R2;
}

void GeometricCSpaceOMPLSolidCylinder::print(std::ostream& out) const
{
    out << "SolidCylinder Space";
}

bool GeometricCSpaceOMPLSolidCylinder::IsPlanar(){
    return false;
}

Vector3 GeometricCSpaceOMPLSolidCylinder::ProjectToVector3(double u, double r, double h)
{
    Config q = ProjectToConfig(u, r, h);
    double x = q[0];
    double y = q[1];
    double z = q[2];
    Vector3 vector(x,y,z);
    return vector;
}

double GeometricCSpaceOMPLSolidCylinder::OMPLStateToSO2Value(const ob::State *qompl)
{
    const ob::SO2StateSpace::StateType *qomplSO2 = 
      qompl->as<ob::CompoundState>()->as<ob::SO2StateSpace::StateType>(0);

    return qomplSO2->value;
}

double GeometricCSpaceOMPLSolidCylinder::OMPLStateToRValue(const ob::State *qompl)
{
    const ob::RealVectorStateSpace::StateType *qomplRnSpace = 
      qompl->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(1);

    return qomplRnSpace->values[0];
}

double GeometricCSpaceOMPLSolidCylinder::OMPLStateToHeightValue(const ob::State *qompl)
{
    const ob::RealVectorStateSpace::StateType *qomplRnSpace = 
      qompl->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(1);

    return qomplRnSpace->values[1];
}

Config GeometricCSpaceOMPLSolidCylinder::OMPLStateToConfig(const ob::State *x)
{
    double u = OMPLStateToSO2Value(x);
    double r = OMPLStateToRValue(x);
    double h = OMPLStateToHeightValue(x);
    return ProjectToConfig(u, r, h);
}

Vector3 GeometricCSpaceOMPLSolidCylinder::getXYZ(const ob::State *x)
{
    double u = OMPLStateToSO2Value(x);
    double r = OMPLStateToRValue(x);
    double h = OMPLStateToHeightValue(x);
    return ProjectToVector3(u, r, h);
}

void GeometricCSpaceOMPLSolidCylinder::DrawGL(GUIState& state)
{
  glDisable(GL_LIGHTING);
  glEnable(GL_BLEND);

  GLDraw::setColor(black);
  glLineWidth(2);

  const double dstep = 0.01;

  //Bottom Circle (inner)
  glBegin(GL_LINE_LOOP);
  for(double d = 0; d < 2*M_PI; d+=dstep){
    Vector3 v = ProjectToVector3(d, radiusInner_, 0);
    GLDraw::glVertex3v(v);
  }
  glEnd();

  //Bottom Circle (outer)
  glBegin(GL_LINE_LOOP);
  for(double d = 0; d < 2*M_PI; d+=dstep)
  {
    Vector3 v = ProjectToVector3(d, radiusOuter_, 0);
    GLDraw::glVertex3v(v);
  }
  glEnd();
  //Top Circle (inner)
  glBegin(GL_LINE_LOOP);
  for(double d = 0; d < 2*M_PI; d+=dstep){
    Vector3 v = ProjectToVector3(d, radiusInner_, height_);
    GLDraw::glVertex3v(v);
  }
  glEnd();

  //Top Circle (outer)
  glBegin(GL_LINE_LOOP);
  for(double d = 0; d < 2*M_PI; d+=dstep)
  {
    Vector3 v = ProjectToVector3(d, radiusOuter_, height_);
    GLDraw::glVertex3v(v);
  }
  glEnd();
  //height
  const double lstep = 2*M_PI/32.0;
  glBegin(GL_LINES);
  for(double d = 0; d < 2*M_PI; d+=lstep)
  {
    Vector3 v = ProjectToVector3(d, radiusOuter_, 0);
    Vector3 u = ProjectToVector3(d, radiusOuter_, height_);
    GLDraw::glVertex3v(v);
    GLDraw::glVertex3v(u);
    Vector3 vInner = ProjectToVector3(d, radiusInner_, 0);
    Vector3 uInner = ProjectToVector3(d, radiusInner_, height_);
    GLDraw::glVertex3v(vInner);
    GLDraw::glVertex3v(uInner);
  }
  glEnd();

  glDisable(GL_BLEND);
  glEnable(GL_LIGHTING);
}

void GeometricCSpaceOMPLSolidCylinder::ConfigToOMPLState(const Config &q, ob::State *qompl)
{
    double x = q[0];
    double y = q[1];
    double h = q[2];

    double r = sqrt(x*x + y*y);
    double u = atan2(y/r, x/r);

    ob::SO2StateSpace::StateType *qomplSO2 = 
      qompl->as<ob::CompoundState>()->as<ob::SO2StateSpace::StateType>(0);

    ob::RealVectorStateSpace::StateType *qomplRnSpace = 
      qompl->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(1);

    qomplSO2->value = u;
    qomplRnSpace->values[0] = r;
    qomplRnSpace->values[1] = h;
}

Config GeometricCSpaceOMPLSolidCylinder::ProjectToConfig(double u, double r, double h)
{
    Config q;q.resize(robot->q.size());q.setZero();

    q[0] = r*cos(u);
    q[1] = r*sin(u);
    q[2] = h + zOffset_;
    return q;
}

