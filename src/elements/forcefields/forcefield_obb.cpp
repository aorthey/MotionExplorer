#include "forcefield_obb.h"
#include "gui/drawMotionPlanner.h"
#include "gui/colors.h"

using namespace Math3D;
using namespace GLDraw;
ForceFieldTypes OrientedBoundingBoxForceField::type()
{
  return OBB;
}
OrientedBoundingBoxForceField::OrientedBoundingBoxForceField(double _power, Vector3 _center, Vector3 _direction, Vector3 _extension):
  center(_center), extension(_extension), power(_power)
{
  for(uint k = 0; k < 3; k++){
    if(extension[k] < 0){
      std::cout << "[OrientedBoundingBoxForceField] extension needs to be non-negative. But it is " << extension << std::endl;
      throw "Negative box extension.";
    }
  }
  Vector3 ex(1,0,0);

  std::vector<Vector3> a,b;
  a.push_back(ex);

  Vector3 normdir = _direction/_direction.norm();
  b.push_back(normdir);

  double err = Math3D::RotationFit(a, b, R);

  if(err > 0.01){
    std::cout << "Error: large rotation error in OrientedBoundingBoxForceField" << std::endl;
    std::cout << "ex: " << ex << std::endl;
    std::cout << "R*ex: " << R << std::endl;
    std::cout << "direction: " << _direction << std::endl;
    std::cout << "ndirection: " << normdir << std::endl;
    throw "Rotation error.";
  }
  //std::cout << R << std::endl;

  force = _power*_direction;
}

bool OrientedBoundingBoxForceField::IsInsideBox( const Vector3 &position )
{
  Vector3 rel;
  R.mulTranspose(position - center, rel);

  for(int i = 0; i < 3; i++){
    if(rel[i] < -extension[i]/2) return false;
    if(rel[i] > extension[i]/2) return false;
  }
  return true;
}

Math3D::Vector3 OrientedBoundingBoxForceField::getForce(const Math3D::Vector3& position, const Math3D::Vector3& velocity)
{
  if(IsInsideBox(position)){
    return force;
  }else{
    return Vector3(0,0,0);
  }
}

void OrientedBoundingBoxForceField::Print(std::ostream &out) const
{
  out << "OrientedBoundingBoxForceField : force " << force  << " center " << center << " extension " << extension;
}

double OrientedBoundingBoxForceField::GetPower(){
  return power;
}
Math3D::Vector3 OrientedBoundingBoxForceField::GetCenter(){
  return center;
}
Math3D::Vector3 OrientedBoundingBoxForceField::GetExtension(){
  return extension;
}
Math3D::Matrix3 OrientedBoundingBoxForceField::GetRotation(){
  return R;
}
void OrientedBoundingBoxForceField::DrawGL(GUIState &state)
{
  glDisable(GL_LIGHTING);
  glEnable(GL_BLEND); 

  //double power = fb->GetPower();
  //Vector3 center = fb->GetCenter();
  //Vector3 extension = fb->GetExtension();
  //Matrix3 R = fb->GetRotation();
  
  glPushMatrix();
  setColor(cForce);
  glTranslate(center);
  Matrix4 RH(R);
  RH(3,3) = 1;
  glMultMatrixd(RH);

  glPointSize(5);
  drawPoint(Vector3(0,0,0));

  glLineWidth(2);

  double ystep = 0.5;
  double zstep = 0.5;
  for(double y = -extension[1]/2; y < extension[1]/2; y+=ystep){
    for(double z = -extension[2]/2; z < extension[2]/2; z+=zstep){

      double x = extension[0]/2;

      glBegin(GL_LINES);
      glVertex3f(-x,y,z);
      glVertex3f(x,y,z);
      glEnd();

      double length = 0.1;
      Vector3 direction(length,0,0);
      if(power < 0) {
        direction *= -1;
      }
      glPushMatrix();
      Vector3 middle(-x/2,y,z);
      glTranslate(middle);
      GLDraw::drawCone(direction, length/2);
      glPopMatrix();
      glPushMatrix();
      middle = Vector3(x/2,y,z);
      glTranslate(middle);
      GLDraw::drawCone(direction, length/2);
      glPopMatrix();
    }
  }


  glPopMatrix();
  glEnable(GL_LIGHTING);
  glDisable(GL_BLEND); 

}
