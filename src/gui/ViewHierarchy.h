#pragma once

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



/** @brief OpenGL Tree Plotting
 */
class ViewHierarchy
{
 public:
  ViewHierarchy();
  void PushLevel(int nodes, std::string robot_name);
  void PopLevel();
  void UpdateSelectionPath( std::vector<int> path );
  void DrawGL();
  int GetLevel();
  void Clear();

  int x,y,heightPerLevel,width;
  int width_column1;
  int width_column2;
  int width_column3;

  double node_radius;
  double node_radius_unselected;
  double node_radius_selected;

  bool drawBound,drawPlotArea;
  GLDraw::GLColor boundColor, plotAreaColor, textColor, nodeColor, nodeSelectedColor, nodeUnselectedColor;

  void DrawNode( double x, double y, double radius, int number = 0);
  void DrawNode( double x, double y, double radius, std::string nstr);
  void DrawLineFromNodeToNode(double x1, double y1, double n1, double x2, double y2, double n2);

 private:
  std::vector<int> level_nodes;
  std::vector<std::string> level_robot_name;
  std::vector<int> selected_path;
  
};
