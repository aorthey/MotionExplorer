#include "forcefield_random.h"

using namespace Math3D;
//##############################################################################
// UniformRandomForceField
//##############################################################################

UniformRandomForceField::UniformRandomForceField(Vector3 _minforce, Vector3 _maxforce):
  minforce(_minforce), maxforce(_maxforce)
{
}

Math3D::Vector3 UniformRandomForceField::getForce(const Math3D::Vector3& position, const Math3D::Vector3& velocity)
{
  Vector3 F;
  for(int i = 0; i < 3; i++){
    F[i] = Math::Rand(minforce[i], maxforce[i]);
  }
  return F;
}

void UniformRandomForceField::print()
{
  std::cout << "UniformRandomForceField  : minforce "<<minforce << " maxforce " << maxforce << std::endl;
}
ForceFieldTypes UniformRandomForceField::type(){
  return UNIFORM_RANDOM;
}
//##############################################################################
// Gaussianrandomforcefield
//##############################################################################

GaussianRandomForceField::GaussianRandomForceField(Vector3 _mean, Vector3 _std):
  mean(_mean), stddeviation(_std)
{
}

Math3D::Vector3 GaussianRandomForceField::getForce(const Math3D::Vector3& position, const Math3D::Vector3& velocity)
{
  Vector3 F;
  for(int i = 0; i < 3; i++){
    F[i] = Math::RandGaussian(mean[i], stddeviation[i]);
  }
  return F;
}

void GaussianRandomForceField::print()
{
  std::cout << "GaussianRandomForceField  : mean "<< mean << " stddev " << stddeviation << std::endl;
}
ForceFieldTypes GaussianRandomForceField::type()
{
  return GAUSSIAN_RANDOM;
}
