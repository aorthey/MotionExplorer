#include <cstddef>
#include <stdio.h>
#undef PPL_HAVE_TYPEOF
#include "3rdparty/ppl.hh"

using namespace Parma_Polyhedra_Library;

int main(int argc,const char** argv) {
  Variable x(0);
  C_Polyhedron ph(1);
  ph.refine_with_constraint( x <= 3.7);
  ph.refine_with_constraint( x >= 0.3);

  Generator_System gs = ph.generators();
  for(Generator_System::const_iterator it = gs.begin(); it != gs.end(); it++) {
    const Generator& g = *it;
    double a =  raw_value(g.coefficient(x)).get_d();
    std::cout << a << "," << g.coefficient(x) << std::endl;
  }
  return 0;
}

