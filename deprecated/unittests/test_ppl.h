#pragma once
#include <cstddef>
#include <stdio.h>
#undef PPL_HAVE_TYPEOF
#include "3rdparty/ppl.hh"
#include "test_util.h"

using namespace Parma_Polyhedra_Library;

TEST(PPLTest, 1DPolyhedron) {
  Variable x(0);
  C_Polyhedron ph(1);
  ph.refine_with_constraint( x <= 3.7);
  ph.refine_with_constraint( x >= 0.3);

  Generator_System gs = ph.generators();

  std::vector<double> values;
  for(Generator_System::const_iterator it = gs.begin(); it != gs.end(); it++) {
    const Generator& g = *it;
    double a =  raw_value(g.coefficient(x)).get_d();
    values.push_back(a);
    //std::cout << a << "," << g.coefficient(x) << std::endl;
  }
  ASSERT_TRUE(values.at(0) == 3.7) << "value is " << values.at(0) << " but should be " << 3.7;
}
