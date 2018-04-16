#pragma once
#include "convex_polyhedron.h"
#include "gui/gui_state.h"
#include <vector>
#include <Eigen/Core>

#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Nef_polyhedron_3.h>

//typedef CGAL::Exact_predicates_exact_constructions_kernel   Kernel;
// typedef Kernel::Point_3                                           Point_3;
// typedef Kernel::Plane_3                                           Plane_3;
typedef CGAL::Nef_polyhedron_3<Kernel, CGAL::SNC_indexed_items>   Nef_polyhedron_3;
class CSpaceOMPL;

class NefPolyhedron{
  public:
    NefPolyhedron() = delete;
    NefPolyhedron( const ConvexPolyhedron* );

    void SubtractObstacles(CSpaceOMPL *cspace);
    std::vector<ConvexPolyhedron> GetConvexDecomposition();

    void DrawGL(GUIState&);
  private:
    Nef_polyhedron_3 *poly;
};
