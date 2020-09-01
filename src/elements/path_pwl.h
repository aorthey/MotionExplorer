#pragma once
#include "gui/gui_state.h"
#include "gui/colors.h"
#include <ompl/geometric/PathGeometric.h>
#include <ompl/control/PathControl.h>
#include <Library/KrisLibrary/math/vector.h>
#include <Library/KrisLibrary/math3d/primitives.h>
#include <Library/KrisLibrary/utils/SmartPointer.h>
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
    PathPiecewiseLinear() = delete;
    PathPiecewiseLinear(CSpaceOMPL *cspace);
    PathPiecewiseLinear(ob::PathPtr p, CSpaceOMPL *cspace, CSpaceOMPL *quotient_space);
    void SetDefaultPath();

    ob::PathPtr GetOMPLPath() const;

    Config Eval(const double t) const;
    Config EvalStates(std::vector<ob::State*> states, const double t) const;
    Config EvalVelocity(const double t) const;
    Vector3 EvalVec3(const double t) const;
    Vector3 EvalVec3(const double t, int ridx) const;

    void StatesToMilestones(
        const std::vector<ob::State*> &states,
        std::vector<Vector3> &milestones,
        int ridx,
        double percentage);

    CSpaceOMPL *GetSpace() const;

    void Normalize(); // convert path length [0,L] -> [0,1]
    void Smooth(bool forceSmoothing=false);

    std::vector<double> GetLengthVector() const;
    int GetNumberOfMilestones();
    double GetLength() const;

    double linewidth{1};
    double widthBorder{0.1};
    double ptsize{1};
    double zOffset{-0.1};
    GLColor cVertex{magenta}, cLine{magenta};
    GLColor cSmoothed{magenta}, cUnsmoothed{red};
    GLColor cRobotVolume{grey};
    GLColor cCross{green};

    void setColor(const GLColor &color);
    bool drawSweptVolume{true};
    bool drawCross{true};

    void DrawGL(GUIState& state);
    void DrawGL(GUIState& state, double t);
    void DrawGLPathPtr(GUIState& state, ob::PathPtr);
    bool draw_planar{false};
    std::vector<double> GetHighCurvatureConfigurations();

    bool Load(const char *fn);
    bool Load(TiXmlElement* node);
    bool Save(const char *fn);
    bool Save(TiXmlElement* node);
    friend std::ostream& operator<< (std::ostream& out, const PathPiecewiseLinear& pwl);

    void SendToController(SmartPointer<RobotController> controller);

  protected:
    double length{0};
    std::vector<double> interLength;//interLength(i) length towards next milestone point from q(i)

    bool isSmooth{false};
    Vector3 Vector3FromState(ob::State *s);
    Vector3 Vector3FromState(ob::State *s, int ridx);
    void Draw2DArrow(Vector3 arrow_pos, Vector3 arrow_dir, double arrow_size_head, double arrow_size_length);
    Vector3 GetNearestStateToTipOfArrow(Vector3 arrow_pos, 
        const std::vector<ob::State*> states, uint k_start_state, double arrow_size_length, int ridx);

    void DrawGLRibbon(const std::vector<ob::State*> &states, double percentage = 1.0);
    void DrawGLRibbonRobotIndex(const std::vector<ob::State*> &states, int ridx, double percentage = 1.0);
    void DrawGLArrowMiddleOfPath(const std::vector<ob::State*> &states, int ridx);
    void DrawGLCross(const std::vector<ob::State*> &states, int ridx);

    CSpaceOMPL *cspace{nullptr};
    CSpaceOMPL *quotient_space{nullptr};

    ob::PathPtr path{nullptr};
    ob::PathPtr path_raw{nullptr};
};
