#include <cstddef>
#include <stdio.h>
#undef PPL_HAVE_TYPEOF
#include "3rdparty/ppl.hh"

using namespace Parma_Polyhedra_Library;

int main(int argc, char* argv[]) 
{
  Variable x(0);
  C_Polyhedron ph(1);

  double Llimit = 0.3;
  double Ulimit = 3.7;

  ph.refine_with_constraint( x >= Llimit);
  ph.refine_with_constraint( x <= Ulimit);

  Generator_System gs = ph.generators();

  std::vector<double> values;
  for(Generator_System::const_iterator it = gs.begin(); it != gs.end(); it++) {
    const Generator& g = *it;
    double a =  raw_value(g.coefficient(x)).get_d();
    values.push_back(a);
    std::cout << a << "," << g.coefficient(x) << std::endl;
  }
}
