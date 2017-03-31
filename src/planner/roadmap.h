#pragma once
#include <vector>
#include <pair>

class Roadmap
{
  Roadmap();
  //FromOMPL();
  static FromKlamptKinodynamicTree(const KinodynamicTree& tree, CSpace *base);

  std::vector<Vector3> GetVertices();
  std::vector<std::pair<Vector3,Vector3> > GetEdges();
  SerializeTree _tree;
}

