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

void UniformRandomForceField::Print(std::ostream &out) const
{
  out << "UniformRandomForceField  : minforce "<<minforce << " maxforce " << maxforce;
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

void GaussianRandomForceField::Print(std::ostream& out) const
{
  out << "GaussianRandomForceField  : mean "<< mean << " stddev " << stddeviation;
}
ForceFieldTypes GaussianRandomForceField::type()
{
  return GAUSSIAN_RANDOM;
}
