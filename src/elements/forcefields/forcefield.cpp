#include "forcefield.h"

using namespace Math3D;
void ForceField::DrawGL(GUIState &state)
{

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
