#pragma once
#include <vector>
#include <tinyxml.h>
#include <KrisLibrary/math/vector.h>
#include <KrisLibrary/math3d/primitives.h>

using namespace Math;
using namespace Math3D;
struct SerializedTreeNode{
  Vector position; //\in R^6 local chart on SE(3)
  Vector config;
  std::vector<Vector> directions;
  double cost_to_goal;
  Vector3 GetXYZ();
  void SetXYZ(double x, double y, double z);
  bool Save(TiXmlElement *node);
  bool Load(TiXmlElement *node);
};

typedef std::vector< SerializedTreeNode > SerializedTree;

