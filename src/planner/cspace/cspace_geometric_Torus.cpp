#include "planner/cspace/cspace_geometric_Torus.h"
#include "planner/cspace/validitychecker/validity_checker_ompl.h"
#include <ompl/base/spaces/SO2StateSpace.h>
#include "ompl/base/spaces/special/TorusStateSpace.h"
#include "common.h"
#include "gui/colors.h"
#include <KrisLibrary/GLdraw/drawextra.h>
#include <KrisLibrary/GLdraw/GLColor.h>

#include <boost/math/constants/constants.hpp>

using namespace boost::math::double_constants;

const double majorRadius_{1.0};
const double minorRadius_{0.6};

#include <ompl/util/Exception.h>
GeometricCSpaceOMPLTorus::GeometricCSpaceOMPLTorus(RobotWorld *world_, int robot_idx):
  GeometricCSpaceOMPL(world_, robot_idx)
{
}

void GeometricCSpaceOMPLTorus::initSpace()
{
    // ob::StateSpacePtr SO2_1(std::make_shared<ob::SO2StateSpace>());
    // ob::StateSpacePtr SO2_2(std::make_shared<ob::SO2StateSpace>());
    // space = SO2_1 + SO2_2;

    ob::StateSpacePtr T(std::make_shared<ob::TorusStateSpace>(majorRadius_, minorRadius_));
		space = T;
}

void GeometricCSpaceOMPLTorus::print(std::ostream& out) const
{
    out << "Torus Space";
}

bool GeometricCSpaceOMPLTorus::IsPlanar(){
    return false;
}

Config GeometricCSpaceOMPLTorus::AnglesToConfig(double u, double v)
{
    const double &R = majorRadius_;
    const double &r = minorRadius_;

    Config q;q.resize(robot->q.size());q.setZero();

    q[0] = (R + r*cos(v))*cos(u);
    q[1] = (R + r*cos(v))*sin(u);
    q[2] = r*sin(v);

    return q;
}

Config GeometricCSpaceOMPLTorus::OMPLStateToConfig(const ob::State *x)
{
    // const ob::SO2StateSpace::StateType *SO2_1 = 
    //   x->as<ob::CompoundState>()->as<ob::SO2StateSpace::StateType>(0);
    // const ob::SO2StateSpace::StateType *SO2_2 = 
    //   x->as<ob::CompoundState>()->as<ob::SO2StateSpace::StateType>(1);
    // const double u = SO2_1->value;
    // const double v = SO2_2->value;

    const ob::TorusStateSpace::StateType *T = 
      x->as<ob::TorusStateSpace::StateType>();
    const double u = T->getS1();
    const double v = T->getS2();

    return AnglesToConfig(u, v);
}

void GeometricCSpaceOMPLTorus::ConfigToOMPLState(const Config &q, ob::State *qompl)
{

    double x = q[0];
    double y = q[1];
    double z = q[2];

    double rxy = sqrt(x*x + y*y);
    double u = atan2(y/rxy, x/rxy);

    const double &R = majorRadius_;

    Vector3 qv(x, y, z);
    Vector3 a(R*cos(u), R*sin(u), 0);
    Vector3 b = qv - a;

    a.inplaceNormalize();
    b.inplaceNormalize();

    double v = acos(dot(a, b));

    // ob::SO2StateSpace::StateType *SO2_1 = 
    //   qompl->as<ob::CompoundState>()->as<ob::SO2StateSpace::StateType>(0);
    // ob::SO2StateSpace::StateType *SO2_2 = 
    //   qompl->as<ob::CompoundState>()->as<ob::SO2StateSpace::StateType>(1);
    // SO2_1->value = u;
    // SO2_2->value = v;
    ob::TorusStateSpace::StateType *T = 
      qompl->as<ob::TorusStateSpace::StateType>();
		T->setS1S2(u, v);
}


Vector3 GeometricCSpaceOMPLTorus::getXYZ(const ob::State *x)
{
    Config q =OMPLStateToConfig(x);
    Vector3 v(q[0], q[1], q[2]);
    return v;
}

void GeometricCSpaceOMPLTorus::DrawGL(GUIState& state)
{
  glDisable(GL_LIGHTING);
  glEnable(GL_BLEND);

  GLDraw::setColor(black);
  glLineWidth(1);

  //Vertical Circles

  //Anglestoconfig(u,v): u pos along inner circle, v pos along outer circle

  const double step = 2*M_PI/32;

  for(double v = 0; v < 2*M_PI; v+=step){
    glBegin(GL_LINE_LOOP);
    for(double u = 0; u < 2*M_PI; u+=0.01){
      Config q = AnglesToConfig(u, v);
      Vector3 x(q[0],q[1],q[2]);
      GLDraw::glVertex3v(x);
    }
    glEnd();
  }

  for(double u = 0; u < 2*M_PI; u+=step){
    glBegin(GL_LINE_LOOP);
    for(double v = 0; v < 2*M_PI; v+=0.01){
      Config q = AnglesToConfig(u, v);
      Vector3 x(q[0],q[1],q[2]);
      GLDraw::glVertex3v(x);
    }
    glEnd();
  }

  glDisable(GL_BLEND);
  glEnable(GL_LIGHTING);
}


