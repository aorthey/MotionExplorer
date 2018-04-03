// #include <ppl.hh>
// #include <stdio.h>
// #include <string.h>

// #include <gmp.h>
// extern "C"{
// #define GMP
// #include <lrslib.h>
// #undef GMP
// }

// #undef PPL_HAVE_TYPEOF
// #include "ppl.hh"

// using namespace Parma_Polyhedra_Library;
// using namespace Eigen;

// struct Floating_Real_Open_Interval_Info_Policy {
//   const_bool_nodef(store_special, false);
//   const_bool_nodef(store_open, true);
//   const_bool_nodef(cache_empty, true);
//   const_bool_nodef(cache_singleton, true);
//   const_bool_nodef(cache_normalized, false);
//   const_int_nodef(next_bit, 0);
//   const_bool_nodef(may_be_empty, true);
//   const_bool_nodef(may_contain_infinity, false);
//   const_bool_nodef(check_empty_result, false);
//   const_bool_nodef(check_inexact, false);
// };

// typedef Interval_Info_Bitset<unsigned int,
//                              Floating_Real_Open_Interval_Info_Policy> Floating_Real_Open_Interval_Info;

// //! The type of an interval with floating point boundaries.
// typedef Interval<double, Floating_Real_Open_Interval_Info> FP_Interval;
// typedef Linear_Form<FP_Interval> FP_Linear_Form;

// using namespace Parma_Polyhedra_Library::IO_Operators;

// Eigen::MatrixXd getGenerators(Eigen::MatrixXd A, Eigen::VectorXd b){
//   std::cout << A << std::endl;
//   std::cout << b << std::endl;
//   std::cout << std::string(80, '-') << std::endl;
  
//   Variable x(0);
//   C_Polyhedron ph(1);
//   ph.refine_with_constraint( x <= 3.7);
//   ph.refine_with_constraint( x >= 0.3);


//   Generator_System gs = ph.generators();
//   for(Generator_System::const_iterator it = gs.begin(); it != gs.end(); it++) {
//     const Generator& g = *it;
//     //double a =  raw_value(g.coefficient(x)).get_d();
//     std::cout << g.coefficient(x) << std::endl;
//   }

//   //PROBLEM 1: how to get floating point generators (solved by using //FP_LINEAR_FORM?)
//   exit(0);


//   //PPL objects have dual structure: constraint_system and generators.
//   //In this case we have the halfspace description hrep as the constraint system
//   //and the vertex description vrep as the generators.
//   const int dim = A.cols();
//   std::vector<Variable> vars;
//   for (int i=0; i < dim; i++) {
//     Variable v(i);
//     vars.push_back(v);
//   }
//   C_Polyhedron ppl_polyhedron(dim);

//   //http://www.bugseng.com/products/ppl/documentation/user/ppl-user-1.2-html/classParma__Polyhedra__Library_1_1Linear__Expression.html
//   for (int i=0; i < A.rows(); i++){
//     FP_Linear_Form lhs;
//     for (int j=0; j < dim; j++){
//       std::cout << " " << A(i,j) << "*x_"<<j << (j<dim-1?" + ":"");
//       lhs += FP_Linear_Form(A(i,j) * vars[j]);
//     }
//     std::cout << std::endl;
//     FP_Linear_Form rhs(b(i) + 0.0*vars[0]);
//     ppl_polyhedron.refine_with_linear_form_inequality(lhs, rhs);
//   }

//   if(ppl_polyhedron.is_bounded()){
//     std::cout << "Polyhedron is bounded" << std::endl;
//   }

//   auto generators = ppl_polyhedron.minimized_generators();
//   generators.print();
//   std::cout << generators.space_dimension() << std::endl;
//   //PROBLEM 2: why are generators zero?

//   for (auto gen = generators.begin(); gen != generators.end(); ++gen) {
//     //float x = Floating_Point_Expression<FP_Interval, FP_Linear_Form>(gen->coefficient(vars[0]));
//     //std::cout << x << std::endl;
//     gen->print();
//     //for (int i=0; i < dim; i++) {
//       //printf("%f", *static_cast<const GMP_Integer>(gen->coefficient(vars[i])));
//     //}
//     //printf("\n");
//     // printf(" %d\n", static_cast<const int> gen->divisor());
//   }
//   exit(0);

//   //ppl_polyhedron.constraints().print();
//   MatrixXd V;
//   return V;
// }

//   // for (int i=0; i < A.rows(); i++) {
//   //   FP_Linear_Form expr;
//   //   for (int j=0; j < dim; j++) {
//   //     expr += FP_Linear_Form(A(i,j) * vars[j]);
//   //   }
//   //   expr.print();
//   //   printf("\n");
//   //   FP_Linear_Form right(b(i) + 0.0 * vars[0]);
//   //   right.print();
//   //   printf("\n");
//   //   ppl_polyhedron.refine_with_linear_form_inequality(expr, right);
//   //   exit(0);
//   //   // ppl_polyhedron.add_constraint(expr <= Linear_Form<double>(self->getB()(i)));
//   // }






#include "vrep.h"
#include "Polyhedron.h"
#include <iostream>


VertexRepresentation::VertexRepresentation(Eigen::MatrixXd A, Eigen::VectorXd b):
  A_eigen(A), b_eigen(b)
{
}
Eigen::MatrixXd VertexRepresentation::GetVertices()
{

  // std::cout << std::string(80, '-') << std::endl; 
  // std::cout << A_eigen << std::endl;
  // std::cout << b_eigen << std::endl;
  // std::cout << std::string(80, '-') << std::endl; 
  Eigen::Polyhedron P;
  P.vrep(A_eigen, b_eigen);
  std::pair<Eigen::MatrixXd, Eigen::VectorXd> VV;
  VV = P.vrep();

  Eigen::MatrixXd V = VV.first;
  return V;
}
