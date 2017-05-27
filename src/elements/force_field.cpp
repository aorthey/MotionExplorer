#include "force_field.h"

using namespace Math3D;
UniformForceField::UniformForceField(Vector3 _force){
  force = _force;
}
Vector3 UniformForceField::getForceAtPosition(Vector3 position){
  return force;
}

void UniformForceField::print(){
  std::cout << "UniformForceField : force " << force << std::endl;
}
ForceFieldTypes UniformForceField::type(){
  return ForceFieldTypes.UNIFORM;
}
RadialForceField::RadialForceField(Vector3 _source, double _power, double _radius):
  source(_source), power(_power), maximum_radius(_radius), minimum_radius(0.1)
{
}

Vector3 RadialForceField::getForceAtPosition(Vector3 position){
  Vector3 relative_position = position - source;

  Vector3 F(0,0,0);

  double dist = relative_position.normSquared();
  if( minimum_radius <= dist && dist <= maximum_radius){
    F = (power/(dist*dist))*(-relative_position);
  }

  return F;
}

void RadialForceField::print(){
  std::cout << "RadialForceField  : source:" << source << " power: "<< power << " radius "<< maximum_radius << std::endl;
}

ForceFieldTypes RadialForceField::type(){
  return ForceFieldTypes.RADIAL;
}
