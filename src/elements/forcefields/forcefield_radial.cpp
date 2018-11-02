#include "forcefield_radial.h"
#include "gui/drawMotionPlanner.h"
#include "gui/colors.h"

using namespace Math3D;
const double minimumRadiusSingularity = 0.05;

RadialForceField::RadialForceField(Vector3 _source, double _power, double _radius):
  source(_source), power(_power), minimum_radius(minimumRadiusSingularity), maximum_radius(_radius)
{
}

Math3D::Vector3 RadialForceField::getForce(const Math3D::Vector3& position, const Math3D::Vector3& velocity)
{
  Vector3 relative_position = position - source;

  Vector3 F(0,0,0);

  double dist = relative_position.normSquared();
  if( minimum_radius <= dist && dist <= maximum_radius){
    F = (power/(dist))*(relative_position);
  }

  return F;
}

void RadialForceField::Print(std::ostream &out) const
{
  out << "RadialForceField  : source:" << source << " power: "<< power << " radius "<< maximum_radius << " color " << cForce;
}

ForceFieldTypes RadialForceField::type(){
  return RADIAL;
}
Math3D::Vector3 RadialForceField::GetSource(){
  return source;
}
double RadialForceField::GetRadius(){
  return maximum_radius;
}
double RadialForceField::GetPower(){
  return power;
}
void RadialForceField::DrawGL(GUIState &state)
{
  glDisable(GL_LIGHTING);
  glEnable(GL_BLEND); 

  glPushMatrix();
  setColor(cForce);
  glTranslate(source);

  glPointSize(5);
  drawPoint(Vector3(0,0,0));

  glLineWidth(2);

  double theta_step = M_PI/4;
  double gamma_step = theta_step;
  for(double theta = 0; theta < 2*M_PI; theta+=theta_step){
    for(double gamma = 0; gamma < M_PI; gamma+=gamma_step){
      double xs = GetRadius()*sin(theta)*cos(gamma);
      double ys = GetRadius()*sin(theta)*sin(gamma);
      double zs = GetRadius()*cos(theta);

      glBegin(GL_LINES);
      glVertex3f(0,0,0);
      glVertex3f(xs,ys,zs);
      glEnd();

      Vector3 middle(xs/2,ys/2,zs/2);
      glPushMatrix();
      glTranslate(middle);

      Vector3 direction = 0.1*middle; 
      if(power < 0) {
        direction *= -1;
      }
      GLDraw::drawCone(direction, 0.5*direction.norm());
      glPopMatrix();
    }
  }
  glPopMatrix();
  glEnable(GL_LIGHTING);
  glDisable(GL_BLEND); 
}
