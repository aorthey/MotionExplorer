#pragma once
#include "gui/gui_state.h"
#include "klampt.h"
#include <vector>
#include <Eigen/Core>
#include <ompl/util/RandomNumbers.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Convex_hull_3/dual/halfspace_intersection_3.h>

typedef CGAL::Exact_predicates_exact_constructions_kernel         Kernel;
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
    ConvexPolyhedron(const ConvexPolyhedron&);

    //vrep and hrep (CGAL based)
    std::vector<Eigen::VectorXd> vrep();
    std::pair<Eigen::MatrixXd, Eigen::VectorXd> hrep();
    std::pair<Eigen::VectorXd, Eigen::VectorXd> bbox();
    bool IsInside(const Eigen::VectorXd &p);

    const Polyhedron_3& GetCGALPolyhedron() const;
    Polyhedron_3& GetCGALPolyhedronNonConst() const;

    Eigen::MatrixXd GetA() const;
    Eigen::VectorXd GetB() const;
    Eigen::VectorXd GetCenter() const;
    Eigen::VectorXd GetGeometricCenter() const;

    Eigen::VectorXd GetRandomPoint();
    Eigen::VectorXd GetRandomPoint_RejectionSampling();

    friend std::ostream& operator<< (std::ostream& out, const ConvexPolyhedron& cvxp);
    void DrawGL(GUIState&);

  protected:
    Vector3 CGALToVector3(const Point_3 &p) const;
    Eigen::VectorXd CGALToEigen(const Point_3 &p) const;
    Point_3 EigenToCGAL(const Eigen::VectorXd &v) const;
    void test_hrep();

    Polyhedron_3 *poly;

    Eigen::MatrixXd A;
    Eigen::VectorXd b;
    Eigen::VectorXd center;

    Eigen::VectorXd bbox_min;
    Eigen::VectorXd bbox_max;

    uint Nsamples_rejected{0};
    uint Nsamples{0};

    std::vector<Eigen::VectorXd> vertices;
    bool bbox_computed{false};
    bool vrep_computed{false};
    bool hrep_computed{false};
    ompl::RNG rng_;

};
