#pragma once

#include "gui/gui_state.h"
#include "planner/cspace/cspace.h"
#include <vector>
#include <iostream>
#include <string>
#include <stdio.h>
#include <KrisLibrary/GLdraw/drawextra.h>
#include <KrisLibrary/GLdraw/GLColor.h>
#include <KrisLibrary/GLdraw/GLUTString.h>
#include <KrisLibrary/math3d/primitives.h>
#include <KrisLibrary/math/complex.h>
#include <KrisLibrary/math3d/geometry3d.h>
#include <KrisLibrary/math3d/basis.h>
#include <ompl/multilevel/planners/explorer/datastructures/LocalMinimaTree.h>

namespace ompl
{
  namespace multilevel
  {
    OMPL_CLASS_FORWARD(LocalMinimaTree);
    OMPL_CLASS_FORWARD(LocalMinimaNode);
  }
}
class PathPiecewiseLinear;

// Different modes to display: (1) Display complete structure
// (2) Display current selected minimum + siblings
class ViewLocalMinimaTree
{
  public:
    ViewLocalMinimaTree() = delete;
    ViewLocalMinimaTree(ompl::multilevel::LocalMinimaTreePtr, std::vector<CSpaceOMPL*> cspace_levels);

    double pathWidth{0.1};
    double pathBorderWidth{0.005};
    int x_,y_,heightPerLevel,width;
    int width_column1;
    int width_column2;
    int width_column3;

    double node_radius;
    double node_radius_unselected;
    double node_radius_selected;

    bool drawBound,drawPlotArea;

    GLDraw::GLColor colorPathSelected;
    GLDraw::GLColor colorPathUnselected;

    GLDraw::GLColor boundColor;
    GLDraw::GLColor plotAreaColor;
    GLDraw::GLColor textColor;
    GLDraw::GLColor nodeColor;
    GLDraw::GLColor nodeSelectedColor;
    GLDraw::GLColor nodeUnselectedColor;

    PathPiecewiseLinear *getPathSelected();
    void DrawGL(GUIState& state);
    void DrawGLScreen(double x, double y);
    void DrawNode( double x, double y, double radius, int number = 0);
    void DrawNode( double x, double y, double radius, std::string nstr);
    void DrawLineFromNodeToNode(double x1, double y1, double n1, double x2, double y2, double n2);

    void DrawGLNodeUnSelected(GUIState& state, ompl::multilevel::LocalMinimaNode* node);
    void DrawGLNodeSelected(GUIState& state, ompl::multilevel::LocalMinimaNode* node);
  private:
    PathPiecewiseLinear *pathSelected_;
    ompl::multilevel::LocalMinimaTreePtr localMinimaTree_;
    std::vector<CSpaceOMPL*> cspace_levels_;
};
