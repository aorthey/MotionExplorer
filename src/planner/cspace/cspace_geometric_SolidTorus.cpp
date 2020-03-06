#include "planner/cspace/cspace_geometric_SolidTorus.h"
#include "planner/cspace/validitychecker/validity_checker_ompl.h"
#include <ompl/base/spaces/SO2StateSpace.h>
#include "common.h"
#include "gui/colors.h"
#include <KrisLibrary/GLdraw/drawextra.h>
#include <KrisLibrary/GLdraw/GLColor.h>

#include <boost/math/constants/constants.hpp>

using namespace boost::math::double_constants;

#include <ompl/util/Exception.h>

GeometricCSpaceOMPLSolidTorus::GeometricCSpaceOMPLSolidTorus(RobotWorld *world_, int robot_idx):
  GeometricCSpaceOMPL(world_, robot_idx)
{
    radiusMid_ = radiusInner_ + 0.5*(radiusOuter_ - radiusInner_);
}

void GeometricCSpaceOMPLSolidTorus::initSpace()
{
    ob::StateSpacePtr SO2(std::make_shared<ob::SO2StateSpace>());
    ob::StateSpacePtr R2(std::make_shared<ob::RealVectorStateSpace>(2));

    vector<double> low;
    low.push_back(radiusInner_);
    low.push_back(0);
    vector<double> high;
    high.push_back(radiusOuter_);
    high.push_back(pi);

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
Vector3 GeometricCSpaceOMPLSolidTorus::ProjectToVector3(double u, double r, double angle)
{
    Config q = ProjectToConfig(u, r, angle);
    double x = q[0];
    double y = q[1];
    double z = q[2];
    Vector3 vector(x,y,z);
    return vector;
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
double GeometricCSpaceOMPLSolidTorus::OMPLStateToAngleValue(const ob::State *qompl)
{
    const ob::RealVectorStateSpace::StateType *qomplRnSpace = 
      qompl->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(1);

    return qomplRnSpace->values[1];
}

Config GeometricCSpaceOMPLSolidTorus::OMPLStateToConfig(const ob::State *x)
{
    double u = OMPLStateToSO2Value(x);
    double r = OMPLStateToRValue(x);
    double angle = OMPLStateToAngleValue(x);
    return ProjectToConfig(u, r, angle);
}

Vector3 GeometricCSpaceOMPLSolidTorus::getXYZ(const ob::State *x)
{
    double u = OMPLStateToSO2Value(x);
    double r = OMPLStateToRValue(x);
    double angle = OMPLStateToAngleValue(x);
    return ProjectToVector3(u, r, angle);
}

void GeometricCSpaceOMPLSolidTorus::DrawGL(GUIState& state)
{
  glDisable(GL_LIGHTING);
  glEnable(GL_BLEND);

  GLDraw::setColor(black);
  glLineWidth(3);

  const double dstep = 0.01;

  //Inner Circle
  glBegin(GL_LINE_LOOP);
  for(double d = 0; d < 2*M_PI; d+=dstep){
    Vector3 v = ProjectToVector3(d, radiusInner_, 0);
    GLDraw::glVertex3v(v);
  }
  glEnd();

  //Outer Circle
  glBegin(GL_LINE_LOOP);
  for(double d = 0; d < 2*M_PI; d+=dstep)
  {
    Vector3 v = ProjectToVector3(d, radiusOuter_, 0);
    GLDraw::glVertex3v(v);
  }
  glEnd();

  //Vertical Circles
  for(double d = 0; d < 2*M_PI; d+=10*dstep){
    double lstep = 0.01;
    glBegin(GL_LINE_LOOP);
    for(double l = 0; l < 2*M_PI; l+=lstep){
      Vector3 v = ProjectToVector3(d, radiusInner_, l);
      GLDraw::glVertex3v(v);
    }
    glEnd();
  }

  glDisable(GL_BLEND);
  glEnable(GL_LIGHTING);
}

void GeometricCSpaceOMPLSolidTorus::ConfigToOMPLState(const Config &q, ob::State *qompl)
{
    double x = q[0];
    double y = q[1];
    double z = q[2];

    double r = sqrt(x*x + y*y);
    double u = atan2(y/r, x/r);

    ob::SO2StateSpace::StateType *qomplSO2 = 
      qompl->as<ob::CompoundState>()->as<ob::SO2StateSpace::StateType>(0);
    qomplSO2->value = u;

    Vector3 qv(x, y, z);
    Vector3 qmid(radiusMid_*cos(u), radiusMid_*sin(u), 0);
    Vector3 qrend(radiusOuter_*cos(u), radiusOuter_*sin(u), 0);
    Vector3 a = qrend - qmid;
    a.inplaceNormalize();
    Vector3 b = qv - qmid;
    b.inplaceNormalize();

    ob::RealVectorStateSpace::StateType *qomplRnSpace = 
      qompl->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(1);
    double angle = atan2(cross(a,b).norm(), dot(a,b));
    double radius = 0;
    Vector3 dq = qv - qmid;
    if(z < 0)
    {
      angle = M_PI - angle;
      radius = radiusMid_ - dq.norm();
    }else{
      radius = radiusMid_ + dq.norm();
    }
    qomplRnSpace->values[0] = radius;
    qomplRnSpace->values[1] = angle;
}

Config GeometricCSpaceOMPLSolidTorus::ProjectToConfig(double u, double r, double angle)
{
    Config q;q.resize(robot->q.size());q.setZero();

    Vector3 r1(radiusMid_*cos(u), radiusMid_*sin(u), 0);
    Vector3 r2(r*cos(u), r*sin(u), 0);
    Vector3 dr = r2 - r1;

    Vector3 ez(0, 0, 1);
    Vector3 c = cross(r1/r1.norm(), ez);

    AngleAxisRotation R(angle, c);
    Matrix3 M; R.getMatrix(M);
    Vector3 v = M*dr;

    v = r1 + v;

    q[0] = v[0];
    q[1] = v[1];
    q[2] = v[2];
    return q;
}

