#include "forcefield_drag.h"

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
void DragForceField::Print(std::ostream &out) const
{
  out << "DragForceField : viscosity " << viscosity;
}
ForceFieldTypes DragForceField::type(){
  return DRAG;
}
