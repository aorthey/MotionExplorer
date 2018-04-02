#include "vrep.h"
#include <stdio.h>
// #include <ppl.hh>
// #include <stdio.h>
// #include <string.h>

// #include <gmp.h>
// extern "C"{
// #define GMP
// #include <lrslib.h>
// #undef GMP
// }

#undef PPL_HAVE_TYPEOF
#include "ppl.hh"

using namespace Parma_Polyhedra_Library;
using namespace Eigen;

struct Floating_Real_Open_Interval_Info_Policy {
  const_bool_nodef(store_special, false);
  const_bool_nodef(store_open, true);
  const_bool_nodef(cache_empty, true);
  const_bool_nodef(cache_singleton, true);
  const_bool_nodef(cache_normalized, false);
  const_int_nodef(next_bit, 0);
  const_bool_nodef(may_be_empty, true);
  const_bool_nodef(may_contain_infinity, false);
  const_bool_nodef(check_empty_result, false);
  const_bool_nodef(check_inexact, false);
};

typedef Interval_Info_Bitset<unsigned int,
                             Floating_Real_Open_Interval_Info_Policy> Floating_Real_Open_Interval_Info;

//! The type of an interval with floating point boundaries.
typedef Interval<double, Floating_Real_Open_Interval_Info> FP_Interval;
typedef Linear_Form<FP_Interval> FP_Linear_Form;

using namespace Parma_Polyhedra_Library::IO_Operators;

Eigen::MatrixXd getGenerators(Eigen::MatrixXd A, Eigen::VectorXd b){
  std::cout << A << std::endl;
  std::cout << b << std::endl;
  std::cout << std::string(80, '-') << std::endl;
  
  Variable x(0);
  C_Polyhedron ph(1);
  ph.refine_with_constraint( x <= 3.7);
  ph.refine_with_constraint( x >= 0.3);


  Generator_System gs = ph.generators();
  for(Generator_System::const_iterator it = gs.begin(); it != gs.end(); it++) {
    const Generator& g = *it;
    //double a =  raw_value(g.coefficient(x)).get_d();
    std::cout << g.coefficient(x) << std::endl;
  }

  //PROBLEM 1: how to get floating point generators (solved by using //FP_LINEAR_FORM?)
  exit(0);


  //PPL objects have dual structure: constraint_system and generators.
  //In this case we have the halfspace description hrep as the constraint system
  //and the vertex description vrep as the generators.
  const int dim = A.cols();
  std::vector<Variable> vars;
  for (int i=0; i < dim; i++) {
    Variable v(i);
    vars.push_back(v);
  }
  C_Polyhedron ppl_polyhedron(dim);

  //http://www.bugseng.com/products/ppl/documentation/user/ppl-user-1.2-html/classParma__Polyhedra__Library_1_1Linear__Expression.html
  for (int i=0; i < A.rows(); i++){
    FP_Linear_Form lhs;
    for (int j=0; j < dim; j++){
      std::cout << " " << A(i,j) << "*x_"<<j << (j<dim-1?" + ":"");
      lhs += FP_Linear_Form(A(i,j) * vars[j]);
    }
    std::cout << std::endl;
    FP_Linear_Form rhs(b(i) + 0.0*vars[0]);
    ppl_polyhedron.refine_with_linear_form_inequality(lhs, rhs);
  }

  if(ppl_polyhedron.is_bounded()){
    std::cout << "Polyhedron is bounded" << std::endl;
  }

  auto generators = ppl_polyhedron.minimized_generators();
  generators.print();
  std::cout << generators.space_dimension() << std::endl;
  //PROBLEM 2: why are generators zero?

  for (auto gen = generators.begin(); gen != generators.end(); ++gen) {
    //float x = Floating_Point_Expression<FP_Interval, FP_Linear_Form>(gen->coefficient(vars[0]));
    //std::cout << x << std::endl;
    gen->print();
    //for (int i=0; i < dim; i++) {
      //printf("%f", *static_cast<const GMP_Integer>(gen->coefficient(vars[i])));
    //}
    //printf("\n");
    // printf(" %d\n", static_cast<const int> gen->divisor());
  }
  exit(0);

  //ppl_polyhedron.constraints().print();
  MatrixXd V;
  return V;
}

  // for (int i=0; i < A.rows(); i++) {
  //   FP_Linear_Form expr;
  //   for (int j=0; j < dim; j++) {
  //     expr += FP_Linear_Form(A(i,j) * vars[j]);
  //   }
  //   expr.print();
  //   printf("\n");
  //   FP_Linear_Form right(b(i) + 0.0 * vars[0]);
  //   right.print();
  //   printf("\n");
  //   ppl_polyhedron.refine_with_linear_form_inequality(expr, right);
  //   exit(0);
  //   // ppl_polyhedron.add_constraint(expr <= Linear_Form<double>(self->getB()(i)));
  // }








VertexRepresentation::VertexRepresentation(Eigen::MatrixXd A, Eigen::VectorXd b):
  A_eigen(A), b_eigen(b)
{
}
Eigen::MatrixXd VertexRepresentation::GetVertices()
{
  // lrs_init ((char*)"\n*vedemo:");
  // lrs_close ((char*)"vedemo:");
  // Constraint_System cs;
  // cs.insert(x >= 0);
  // cs.insert(x <= 3);
  // cs.insert(y >= 0);
  // cs.insert(y <= 3);
  // C_Polyhedron ph(cs);
  // Generator_System gs = ph.generators(); // Use ph.minimized_generators() to minimal set of points for the polytope
  // for(Generator_System::const_iterator it = gs.begin(); it != gs.end(); it++) {
  //   const Generator& g = *it;
  //   std::cout << g;
  // }

  getGenerators(A_eigen, b_eigen);

  Eigen::MatrixXd V;
  return V;
}
// /* vedemo.c     lrslib vertex enumeration demo           */
// /* last modified: May 29, 2001                           */
// /* Copyright: David Avis 2001, avis@cs.mcgill.ca         */
// /* Demo driver for ve code using lrs                 */
// /* This program computes vertices of generated cubes */
// /* given by H-representation                         */

// #include <stdio.h>
// #include <string.h>
// #include "lrslib.h"

// #define MAXCOL 1000     /* maximum number of colums */

// void makecube (lrs_dic *P, lrs_dat *Q);

// int
// main (int argc, char *argv[])

// {
//   lrs_dic *P; /* structure for holding current dictionary and indices  */
//   lrs_dat *Q; /* structure for holding static problem data             */
//   lrs_mp_vector output; /* one line of output:ray,vertex,facet,linearity */
//   lrs_mp_matrix Lin;    /* holds input linearities if any are found      */


//   long i;
//   long col;     /* output column index for dictionary            */

// /* Global initialization - done once */

//   if ( !lrs_init ("\n*vedemo:"))
//     return 1;

// /* compute the vertices of a set of hypercubes given by */
// /* their H-representations.                             */

//   for(i=1;i<=3;i++)
//   {

// /* allocate and init structure for static problem data */

//     Q = lrs_alloc_dat ("LRS globals");
//     if (Q == NULL)
//        return 1;

// /* now flags in lrs_dat can be set */

//     Q->n=i+2;           /* number of input columns         (dimension + 1 )  */
//     Q->m=2*i+2;         /* number of input rows = number of inequalities     */ 

//     output = lrs_alloc_mp_vector (Q->n);

//     P = lrs_alloc_dic (Q);   /* allocate and initialize lrs_dic      */
//     if (P == NULL)
//         return 1;

// /* Build polyhedron: constraints and objective */ 

//     makecube(P,Q);

// /* code from here is borrowed from lrs_main */

// /* Pivot to a starting dictionary                      */

//   if (!lrs_getfirstbasis (&P, Q, &Lin, FALSE))
//     return 1;

// /* There may have been column redundancy               */
// /* (although not for this example of hypercubes)       */

// /* If so the linearity space is obtained and redundant */
// /* columns are removed. User can access linearity space */
// /* from lrs_mp_matrix Lin dimensions nredundcol x d+1  */


//   for (col = 0L; col < Q->nredundcol; col++)  /* print linearity space */
//     lrs_printoutput (Q, Lin[col]);      /* Array Lin[][] holds the coeffs.   */

// /* We initiate reverse search from this dictionary       */
// /* getting new dictionaries until the search is complete */
// /* User can access each output line from output which is */
// /* a vertex/ray/facet from the lrs_mp_vector output      */

//   do
//     {
//       for (col = 0; col <= P->d; col++)
//         if (lrs_getsolution (P, Q, output, col))
//           lrs_printoutput (Q, output);
//     }
//     while (lrs_getnextbasis (&P, Q, FALSE));

//   lrs_printtotals (P, Q);    /* print final totals */

// /* free space : do not change order of next 3 lines! */

//    lrs_clear_mp_vector (output, Q->n);
//    lrs_free_dic (P,Q);           /* deallocate lrs_dic */
//    lrs_free_dat (Q);             /* deallocate lrs_dat */

//   }    /* end of loop for i= ...  */
 
//  lrs_close ("vedemo:");
//  printf("\n");
//  return 0;
// }  /* end of main */

// void
// makecube (lrs_dic *P, lrs_dat *Q)
// /* generate H-representation of a unit hypercube */
// {
// long num[MAXCOL];
// long den[MAXCOL];
// long row, j;
// long m=Q->m;       /* number of inequalities      */
// long n=Q->n;       /* hypercube has dimension n-1 */

// for (row=1;row<=m;row++)
//      {
//       for(j=0;j<n;j++)
//           { num [j] = 0;
//             den [j] = 1;
//           }
//        if (row < n)
//           { num[0] = 1;
//             num[row] = -1;
//           }
//        else
//           { num[0] = 0;
//             num[row+1-n] = 1;
//           }
//        lrs_set_row(P,Q,row,num,den,GE);
//      }

// }

