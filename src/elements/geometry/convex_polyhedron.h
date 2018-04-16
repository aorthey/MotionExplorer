#pragma once
#include "gui/gui_state.h"
#include <vector>
#include <Eigen/Core>

#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Convex_hull_3/dual/halfspace_intersection_3.h>

typedef CGAL::Exact_predicates_exact_constructions_kernel   Kernel;
typedef Kernel::Point_3                                           Point_3;
typedef Kernel::Plane_3                                           Plane_3;
typedef CGAL::Polyhedron_3<Kernel>                                Polyhedron_3;

class ConvexPolyhedron{

  public:

    //Polyhedron is constructed by
    // -- (A,b) hyperplanes
    // -- (A,b,center) hyperplanes + center point (slightly faster computation)

    ConvexPolyhedron() = delete;
    ConvexPolyhedron(Eigen::MatrixXd A_, Eigen::VectorXd b_);
    ConvexPolyhedron(Eigen::MatrixXd A_, Eigen::VectorXd b_, Eigen::VectorXd center_);
    ConvexPolyhedron(Polyhedron_3 &poly_);

    //vrep and hrep (CGAL based)
    std::vector<Eigen::VectorXd> vrep() const;
    std::pair<Eigen::MatrixXd, Eigen::VectorXd> hrep() const;

    const Polyhedron_3& GetCGALPolyhedron() const;
    Polyhedron_3& GetCGALPolyhedronNonConst() const;

    void DrawGL(GUIState&);

  protected:

    Polyhedron_3 *poly;

    Eigen::MatrixXd A;
    Eigen::VectorXd b;
    Eigen::VectorXd center;

    bool convex;

};
