#include "force_field.h"

using namespace Math3D;

//##############################################################################
// UniformForceField
//##############################################################################
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
  return UNIFORM;
}
//##############################################################################
// Radialforcefield
//##############################################################################
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
  std::cout << "RadialForceField  : source:" << source << " power: "<< power << " radius "<< maximum_radius << " color " << color << std::endl;
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
//##############################################################################
// UniformRandomForceField
//##############################################################################

UniformRandomForceField::UniformRandomForceField(Vector3 _minforce, Vector3 _maxforce):
  minforce(_minforce), maxforce(_maxforce){
}

Math3D::Vector3 UniformRandomForceField::getForceAtPosition(Vector3 position){
  Vector3 F;
  for(int i = 0; i < 3; i++){
    F[i] = Math::Rand(minforce[i], maxforce[i]);
  }
  return F;
}

void UniformRandomForceField::print(){
  std::cout << "UniformRandomForceField  : minforce "<<minforce << " maxforce " << maxforce << std::endl;
}
ForceFieldTypes UniformRandomForceField::type(){
  return UNIFORM_RANDOM;
}
//##############################################################################
// Gaussianrandomforcefield
//##############################################################################

GaussianRandomForceField::GaussianRandomForceField(Vector3 _mean, Vector3 _std):
  mean(_mean), stddeviation(_std){
}

Math3D::Vector3 GaussianRandomForceField::getForceAtPosition(Vector3 position){
  Vector3 F;
  for(int i = 0; i < 3; i++){
    F[i] = Math::RandGaussian(mean[i], stddeviation[i]);
  }
  return F;
}

void GaussianRandomForceField::print(){
  std::cout << "GaussianRandomForceField  : mean "<< mean << " stddev " << stddeviation << std::endl;
}
ForceFieldTypes GaussianRandomForceField::type(){
  return GAUSSIAN_RANDOM;
}

//##############################################################################
// OrientedBoundingboxforcefield
//##############################################################################
ForceFieldTypes OrientedBoundingBoxForceField::type(){
  return OBB;
}
OrientedBoundingBoxForceField::OrientedBoundingBoxForceField(double _power, Vector3 _center, Vector3 _direction, Vector3 _extension):
  power(_power), center(_center), extension(_extension)
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

bool OrientedBoundingBoxForceField::IsInsideBox( const Vector3 &position ){
  Vector3 rel;
  R.mulTranspose(position - center, rel);

  for(int i = 0; i < 3; i++){
    if(rel[i] < -extension[i]/2) return false;
    if(rel[i] > extension[i]/2) return false;
  }
  return true;
}

Vector3 OrientedBoundingBoxForceField::getForceAtPosition(Vector3 position){

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
