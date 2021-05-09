#include "planner/cspace/cspace_geometric_Sphere.h"
#include "planner/cspace/validitychecker/validity_checker_ompl.h"
#include <ompl/base/spaces/SO2StateSpace.h>
#include "ompl/base/spaces/special/SphereStateSpace.h"
#include "common.h"
#include "gui/colors.h"
#include <KrisLibrary/GLdraw/drawextra.h>
#include <KrisLibrary/GLdraw/GLColor.h>

#include <boost/math/constants/constants.hpp>

using namespace boost::math::double_constants;

const double radius_{1.0};

#include <ompl/util/Exception.h>
// ob::SpaceInformationPtr GeometricCSpaceOMPLSphere::SpaceInformationPtr()
// {
//     si = BaseT::SpaceInformationPtr();
//     const ob::StateSamplerAllocator allocator = allocSphereStateSampler;
//     si->getStateSpace()->setStateSamplerAllocator(allocator);
//     return si;
// }


GeometricCSpaceOMPLSphere::GeometricCSpaceOMPLSphere(RobotWorld *world_, int robot_idx):
  GeometricCSpaceOMPL(world_, robot_idx)
{
}

void GeometricCSpaceOMPLSphere::initSpace()
{
    ob::StateSpacePtr S(std::make_shared<ob::SphereStateSpace>());
		space = S;
}

void GeometricCSpaceOMPLSphere::print(std::ostream& out) const
{
    out << "Sphere Space";
}

bool GeometricCSpaceOMPLSphere::IsPlanar(){
    return false;
}

Config GeometricCSpaceOMPLSphere::AnglesToConfig(double theta, double phi)
{
    const double &r = radius_;

    Config q;q.resize(robot->q.size());q.setZero();

    q[0] = r*sin(phi)*cos(theta);
    q[1] = r*sin(phi)*sin(theta);
    q[2] = r*cos(phi);

    return q;
}

Config GeometricCSpaceOMPLSphere::OMPLStateToConfig(const ob::State *x)
{
    const ob::SphereStateSpace::StateType *S = 
      x->as<ob::SphereStateSpace::StateType>();
    const double theta = S->getTheta();
    const double phi = S->getPhi();

    // Eigen::Vector3f v = space->as<ob::SphereStateSpace>()->toVector(x);

    return AnglesToConfig(theta, phi);
}

void GeometricCSpaceOMPLSphere::ConfigToOMPLState(const Config &q, ob::State *qompl)
{
    double x = q[0];
    double y = q[1];
    double z = q[2];

    //normalize
    double r = sqrt(x*x + y*y + z*z);
    x = radius_ * x/r;
    y = radius_ * y/r;
    z = radius_ * z/r;

    double theta = atan2(y, x);
    double phi = atan2(sqrtf(x*x + y*y), z);

    ob::SphereStateSpace::StateType *S = 
      qompl->as<ob::SphereStateSpace::StateType>();
		S->setThetaPhi(theta, phi);

    // std::cout << std::string(80, '-') << std::endl;
    // SpaceInformationPtr()->printState(qompl);
}


Vector3 GeometricCSpaceOMPLSphere::getXYZ(const ob::State *x)
{
    Config q =OMPLStateToConfig(x);
    Vector3 v(q[0], q[1], q[2]);
    return v;
}

void GeometricCSpaceOMPLSphere::DrawGL(GUIState& state)
{
  glDisable(GL_LIGHTING);
  glEnable(GL_BLEND);

  GLDraw::setColor(black);
  glLineWidth(1);

  //Vertical Circles

  //Anglestoconfig(u,v): u pos along inner circle, v pos along outer circle

  const double step = 2*M_PI/32;

  for(double phi = 0; phi < 2*M_PI; phi+=step){
    glBegin(GL_LINE_LOOP);
    for(double theta = 0; theta < 2*M_PI; theta+=0.01){
      Config q = AnglesToConfig(theta, phi);
      Vector3 x(q[0],q[1],q[2]);
      GLDraw::glVertex3v(x);
    }
    glEnd();
  }
  for(double theta = 0; theta < 2*M_PI; theta+=step){
    glBegin(GL_LINE_LOOP);
    for(double phi = 0; phi < 2*M_PI; phi+=0.01){
      Config q = AnglesToConfig(theta, phi);
      Vector3 x(q[0],q[1],q[2]);
      GLDraw::glVertex3v(x);
    }
    glEnd();
  }

  glDisable(GL_BLEND);
  glEnable(GL_LIGHTING);
}


