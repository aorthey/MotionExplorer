#include "planner/cspace/cspace_geometric_KleinBottle.h"

#include "planner/cspace/validitychecker/validity_checker_ompl.h"
// #include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/spaces/special/KleinBottleStateSpace.h>
#include "common.h"
#include "gui/colors.h"
#include <KrisLibrary/GLdraw/drawextra.h>
#include <KrisLibrary/GLdraw/GLColor.h>

#include <ompl/util/Exception.h>

GeometricCSpaceOMPLKleinBottle::GeometricCSpaceOMPLKleinBottle(RobotWorld *world_, int robot_idx):
  GeometricCSpaceOMPL(world_, robot_idx)
{
}
GeometricCSpaceOMPLKleinBottle::~GeometricCSpaceOMPLKleinBottle()
{
  if(stateTmp)
  {
    space->freeState(stateTmp);
    space->freeState(state2);
    space->freeState(state1);
  }
}

void GeometricCSpaceOMPLKleinBottle::initSpace()
{
    ob::StateSpacePtr M(std::make_shared<ob::KleinBottleStateSpace>());
		space = M;
    stateTmp = space->allocState();
    state1 = space->allocState();
    state2 = space->allocState();
}

void GeometricCSpaceOMPLKleinBottle::print(std::ostream& out) const
{
    out << "KleinBottle Space";
}

bool GeometricCSpaceOMPLKleinBottle::IsPlanar(){
    return false;
}

void GeometricCSpaceOMPLKleinBottle::ConfigToOMPLState(const Config &q, ob::State *qompl)
{

  //We do not have an explicit inverse formula for Klein bottle. To still be
  //able to project from points to UV states, we search through a set of grid
  //points and find the one which is nearest to the point. Since we use this
  //function only to project start/goal configurations, we do not slow down any
  //algorithm.

    Vector3 p(q[0],q[1],q[2]);

    double d_best = dInf;
    double v_best = 0;
    double u_best = 0;

    double dstep = 0.01;
    for(double v = 0; v < 2*M_PI; v+=2*M_PI/8)
    {
      for(double u = 0; u < M_PI; u+=dstep)
      {
        Vector3 p_uv = ProjectToVector3(u, v);
        double d = (p_uv-p).norm();
        if(d < d_best)
        {
          v_best = v;
          u_best = u;
          d_best = d;
        }
      }
    }

    auto s = qompl->as<ob::KleinBottleStateSpace::StateType>();
    s->setUV(u_best,v_best - M_PI);
}

Vector3 GeometricCSpaceOMPLKleinBottle::ProjectToVector3(double u, double v)
{
    Config q = ProjectToConfig(u, v);
    double x = q[0];
    double y = q[1];
    double z = q[2];
    Vector3 vector(x,y,z);
    return vector;
}

Config GeometricCSpaceOMPLKleinBottle::ProjectToConfig(double u, double v)
{
    auto s = stateTmp->as<ob::KleinBottleStateSpace::StateType>();
    s->setUV(u, v);
    Eigen::Vector3f p = space->as<ob::KleinBottleStateSpace>()->toVector(s);
    Config q;q.resize(robot->q.size());q.setZero();
    q[0] = p[0];
    q[1] = p[1];
    q[2] = p[2];
    return q;
}

void GeometricCSpaceOMPLKleinBottle::DrawGL(GUIState& state)
{
  glDisable(GL_LIGHTING);
  glEnable(GL_BLEND);

  GLDraw::setColor(black);
  glLineWidth(3);

  const double dstep = 0.05;
  //Circular grid lines on bottle
  for(double u = 0; u < M_PI; u+=dstep)
  {
    glBegin(GL_LINE_LOOP);
    for(double v = 0; v < 2*M_PI; v+=2*M_PI/16)
    {
      Vector3 p = ProjectToVector3(u, v);
      GLDraw::glVertex3v(p);
    }
    glEnd();
  }
  //Vertical grid lines on bottle
  glLineWidth(3);
  for(double v = 0; v < 2*M_PI; v+=2*M_PI/8)
  {
    glBegin(GL_LINE_STRIP);
    for(double u = 0; u < M_PI; u+=dstep)
    {
      Vector3 p = ProjectToVector3(u, v);
      GLDraw::glVertex3v(p);
    }
    glEnd();
  }

  drawEdge(+0.2,0, M_PI-0.2, -M_PI);
  drawEdge(+0.2,0, M_PI-0.2, +M_PI);
  drawEdge(+0.2, 0.5*M_PI, M_PI-0.2, 0.5*M_PI);
  drawEdge(+0.2,-0.3*M_PI, M_PI-0.2, -M_PI+0.3*M_PI);
  drawEdge(+0.2,-M_PI, M_PI-0.2, 0);
  drawEdge(+0.2,+M_PI, M_PI-0.2, 0);

  drawEdge(M_PI-0.2,0, +0.2, -M_PI);
  drawEdge(M_PI-0.2,0, +0.2, +M_PI);
  drawEdge(M_PI-0.2, 0.5*M_PI, +0.2, 0.5*M_PI);
  drawEdge(M_PI-0.2,-0.3*M_PI, +0.2, -M_PI+0.3*M_PI);
  drawEdge(M_PI-0.2,-M_PI, +0.2, 0);
  drawEdge(M_PI-0.2,+M_PI, +0.2, 0);

  // drawEdge(+0.2,-M_PI, M_PI-0.2, -M_PI);
  // drawEdge(+0.2,+M_PI, M_PI-0.2, +M_PI);

  //COORDINATE LINES
  // glLineWidth(5);
  // GLDraw::setColor(green);
  // glBegin(GL_LINE_STRIP);
  // for(double u = 0; u < M_PI; u+=0.01)
  // {
  //   Vector3 p = ProjectToVector3(u, 0);
  //   GLDraw::glVertex3v(p);
  // }
  // glEnd();

  glDisable(GL_BLEND);
  glEnable(GL_LIGHTING);

}

void GeometricCSpaceOMPLKleinBottle::drawEdge(double u1, double v1, double u2, double v2)
{
  GLDraw::setColor(red);
  glLineWidth(10);
  glBegin(GL_LINE_STRIP);
  auto K = space->as<ob::KleinBottleStateSpace>();

  auto s = stateTmp->as<ob::KleinBottleStateSpace::StateType>();
  auto s1 = state1->as<ob::KleinBottleStateSpace::StateType>();
  auto s2 = state2->as<ob::KleinBottleStateSpace::StateType>();
  s1->setUV(u1, v1); 
  s2->setUV(u2, v2);
  for(double t = 0; t < 1; t+=0.1)
  {
    K->interpolate(s1, s2, t, s);
    Eigen::Vector3f p = space->as<ob::KleinBottleStateSpace>()->toVector(s);
    Vector3 v;
    v[0] = p[0];
    v[1] = p[1];
    v[2] = p[2];
    GLDraw::glVertex3v(v);
  }
  glEnd();
}

Config GeometricCSpaceOMPLKleinBottle::OMPLStateToConfig(const ob::State *x)
{
    auto p = x->as<ob::KleinBottleStateSpace::StateType>();
    double u = p->getU();
    double v = p->getV();
    return ProjectToConfig(u, v);
}

Vector3 GeometricCSpaceOMPLKleinBottle::getXYZ(const ob::State *x)
{
    auto p = x->as<ob::KleinBottleStateSpace::StateType>();
    double u = p->getU();
    double v = p->getV();
    return ProjectToVector3(u, v);
}
