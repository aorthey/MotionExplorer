#include "forcefield_cylindrical.h"
#include "gui/drawMotionPlanner.h"
#include "gui/colors.h"
#include "KrisLibrary/math3d/basis.h"

using namespace Math3D;
using namespace GLDraw;

const double minimumRadiusSingularity = 0.05;

CylindricalForceField::CylindricalForceField(Math3D::Vector3 _source, Math3D::Vector3 _direction, double _elongation, double _radius, double _power):
  source(_source), direction(_direction), elongation(_elongation), minimum_radius(minimumRadiusSingularity), maximum_radius(_radius), power(_power)
{
  direction /= direction.length();
}

Math3D::Vector3 CylindricalForceField::getForce(const Math3D::Vector3& position, const Math3D::Vector3& velocity)
{
  Vector3 x = position - source;
  Vector3 v = x - (dot(x,direction))*direction;
  //TODO: make it depend on the velocity

  Vector3 F(0,0,0);

  double dist = v.normSquared();
  if( minimum_radius <= dist && dist <= maximum_radius){
    F = power*(1/(2*M_PI*v.norm()))*(cross(v,direction));
  }
  return F;
}

void CylindricalForceField::Print(std::ostream &out) const
{
  out << "CylindricalForceField  : source:" << source << " direction: "<< direction;
  out << " elongation " << elongation << " radius " << maximum_radius;
  out << " power "<< power << " color " << cForce;
}

ForceFieldTypes CylindricalForceField::type(){
  return CYLINDRICAL;
}
Math3D::Vector3 CylindricalForceField::GetSource(){
  return source;
}
Math3D::Vector3 CylindricalForceField::GetDirection(){
  return direction;
}
double CylindricalForceField::GetElongation(){
  return elongation;
}
double CylindricalForceField::GetRadius(){
  return maximum_radius;
}
double CylindricalForceField::GetPower(){
  return power;
}
void CylindricalForceField::DrawGL(GUIState &state)
{
  glDisable(GL_LIGHTING);
  glEnable(GL_BLEND); 

  Vector3 ndirection = direction;
  ndirection /= ndirection.length();

  double radius = GetRadius();

  glPushMatrix();
  setColor(cForce);
  glTranslate(source);

  //draw cable axis
  glPointSize(5);
  drawPoint(Vector3(0,0,0));
  glLineWidth(4);
  Vector3 vv = 0.5*elongation*ndirection;
  glBegin(GL_LINES);
  glVertex3f(-vv[0],-vv[1],-vv[2]);
  glVertex3f(vv[0],vv[1],vv[2]);
  glEnd();

  double verticalDistanceConcentricCircles = 0.2;
  double horizontalDistanceConcentricCircles = 1;

  uint numSteps = 16; 

  for(double dv = -0.5*elongation; dv <= 0.5*elongation; dv+=verticalDistanceConcentricCircles){
    for(double dr = horizontalDistanceConcentricCircles; dr <= radius; dr+=horizontalDistanceConcentricCircles){
        glPushMatrix();
        float inc = fTwoPi/numSteps;
        Vector3 eu,ev;
        GetCanonicalBasis(ndirection,eu,ev);
        Complex x,dx;
        dx.setPolar(One,inc);
     
        glBegin(GL_LINE_LOOP);
        x.set(dr,0);
        for(uint j=0; j<numSteps; j++) {
          glVertex3v(ndirection*dv + x.x*eu+x.y*ev);
          x=x*dx;
        }
        glEnd();

        uint Narrows = 5;
        inc = fTwoPi/Narrows;
        dx.setPolar(One,inc);
        x.set(dr,0);

        for(uint j = 0; j < Narrows; j++){
          glPushMatrix();
          Vector3 rr = x.x*eu + x.y*ev;
          Vector3 arrowS(ndirection*dv + rr);
          x=x*dx;

          glTranslate(arrowS);
          Vector3 coneori = cross(ndirection,rr);
          coneori /= coneori.length();
          double length = 0.1;
          if(power < 0) {
            coneori *= -1;
          }
          GLDraw::drawCone(length*coneori, 0.5*length);
          glPopMatrix();
        }
        glPopMatrix();

    }
  }

  glPopMatrix();
  glEnable(GL_LIGHTING);
  glDisable(GL_BLEND); 
}
