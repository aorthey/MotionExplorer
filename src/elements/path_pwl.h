#pragma once
#include "gui/gui_state.h"
#include "gui/colors.h"
#include "elements/swept_volume.h"
#include <ompl/geometric/PathGeometric.h>
#include <Library/KrisLibrary/math/vector.h>
#include <Library/KrisLibrary/math3d/primitives.h>
typedef Math::Vector Config;
using Math3D::Vector3;

// Continuous Piecewise-Linear Path 
//
// [0,L] -> R^n, piecewise linear
// [0,1] -> R^n, piecewise linear (normalized)
namespace ob = ompl::base;
namespace og = ompl::geometric;

class CSpaceOMPL;
class RobotController;

class PathPiecewiseLinear
{
  public:
    PathPiecewiseLinear();
    PathPiecewiseLinear(CSpaceOMPL *cspace);
    PathPiecewiseLinear(ob::PathPtr p, CSpaceOMPL *cspace);

    Config Eval(const double t) const;
    void Normalize(); // convert path length [0,L] -> [0,1]
    void Smooth();

    std::vector<double> GetLengthVector() const;
    double GetLength() const;
    Vector3 EvalVec3(const double t) const;
    Config EvalMilestone(const int k) const;

    GLColor cVertex{magenta}, cLine{magenta};
    double linewidth{10};
    double ptsize{10};
    virtual void DrawGLPathPtr(ob::PathPtr);

    void DrawGL(GUIState& state);
    bool Load(const char *fn);
    bool Load(TiXmlElement* node);
    bool Save(const char *fn);
    bool Save(TiXmlElement* node);
    friend std::ostream& operator<< (std::ostream& out, const PathPiecewiseLinear& pwl);

    void SendToController(SmartPointer<RobotController> controller);

  protected:
    double length{0};
    std::vector<double> interLength;//interLength(i) length towards next milestone point from q(i)

    SweptVolume *sv{nullptr};
    CSpaceOMPL *cspace;

    ob::PathPtr path;
    ob::PathPtr path_raw;
};
