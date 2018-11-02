#include "forcefield.h"

using namespace Math3D;
void ForceField::DrawGL(GUIState &state)
{

}

std::ostream& operator<< (std::ostream& out, const ForceField& ff) 
{
  ff.Print(out);
  return out;
}
