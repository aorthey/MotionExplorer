#include "forcefield.h"

using namespace Math3D;
void ForceField::DrawGL(GUIState &state)
{

}

//##############################################################################
// OrientedBoundingboxforcefield
//##############################################################################
ForceFieldTypes OrientedBoundingBoxForceField::type()
{
  return OBB;
}
OrientedBoundingBoxForceField::OrientedBoundingBoxForceField(double _power, Vector3 _center, Vector3 _direction, Vector3 _extension):
  center(_center), extension(_extension), power(_power)
{
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
    exit(0);
  }
  std::cout << R << std::endl;

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

void OrientedBoundingBoxForceField::print(){
  std::cout << "OrientedBoundingBoxForceField : force " << force  << " center " << center << " extension " << extension << std::endl;
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
//##############################################################################
// Dragforcefield
//##############################################################################
DragForceField::DragForceField(double viscosity_):
  viscosity(viscosity_)
{
}
Math3D::Vector3 DragForceField::getForce(const Math3D::Vector3& position, const Math3D::Vector3& velocity)
{
  double v = velocity.norm();
  double d = v/(viscosity*viscosity);
  return -d*velocity;
}
void DragForceField::print()
{
  std::cout << "DragForceField : viscosity " << viscosity << std::endl;
}
ForceFieldTypes DragForceField::type(){
  return DRAG;
}
