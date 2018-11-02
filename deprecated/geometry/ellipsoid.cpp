#include "ellipsoid.h"
using namespace GLDraw;

Ellipsoid::Ellipsoid(Eigen::MatrixXd C_, Eigen::VectorXd d_):
  C(C_), d(d_)
{
}

void Ellipsoid::DrawGL(GUIState &state)
{
  if(state("draw_cover_ellipsoid")){
    glLineWidth(1);
    setColor(color);

    Vector3 center(d[0],d[1],d[2]);
    Vector3 u(C(0,0),C(1,0),C(2,0));
    Vector3 v(C(0,1),C(1,1),C(2,1));
    Vector3 w(C(0,2),C(1,2),C(2,2));
    if(filled)
      drawEllipsoid(center, u, v, w);
    else
      drawWireEllipsoid(center, u, v, w);
  }
}
void Ellipsoid::drawWireEllipsoid(Vector3 &c, Vector3 &u, Vector3 &v, Vector3 &w)
{
  //x = a*cos(t) cos(s)
  //y = b*cos(t) sin(s)
  //z = c*sin(t)
  //
  // -pi/2 <= t <= pi/2 
  // -pi <= s <= pi

  float tStep = M_PI/(float)numSteps;
  float sStep = 2*M_PI/(float)numSteps;

  glPushMatrix();
  glLineWidth(2);
  for(float t = -M_PI/2; t <= M_PI/2; t += tStep)
  {
    glBegin(GL_LINE_LOOP);
    for(float s = -M_PI; s <= M_PI; s += sStep)
    {
      Vector3 point = cos(t)*cos(s)*u + cos(t)*sin(s)*v + sin(t)*w + c;
      glVertex3f(point[0],point[1],point[2]);
    }
    glEnd();
  }
  for(float t = -M_PI/2; t <= M_PI/2; t += tStep)
  {
    glBegin(GL_LINE_LOOP);
    for(float s = -M_PI; s <= M_PI; s += sStep)
    {
      Vector3 point = cos(t)*cos(s)*v + cos(t)*sin(s)*w + sin(t)*u + c;
      glVertex3f(point[0],point[1],point[2]);
    }
    glEnd();
  }
  glPopMatrix();
}
void Ellipsoid::drawEllipsoid(Vector3 &c, Vector3 &u, Vector3 &v, Vector3 &w)
{
  //x = a*cos(t) cos(s)
  //y = b*cos(t) sin(s)
  //z = c*sin(t)
  //
  // -pi/2 <= t <= pi/2 
  // -pi <= s <= pi

  float tStep = M_PI/(float)numSteps;
  float sStep = 2*M_PI/(float)numSteps;

  //glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
  for(float t = -M_PI/2; t <= M_PI/2; t += tStep)
  {
    glBegin(GL_TRIANGLE_STRIP);
    for(float s = -M_PI; s <= M_PI; s += sStep)
    {
      Vector3 p1 = cos(t)*cos(s)*u + cos(t)*sin(s)*v + sin(t)*w + c;
      Vector3 p2 = cos(t+tStep)*cos(s)*u + cos(t+tStep)*sin(s)*v + sin(t+tStep)*w + c;
      glVertex3f(p1[0],p1[1],p1[2]);
      glVertex3f(p2[0],p2[1],p2[2]);
    }
    glEnd();
  }
}
