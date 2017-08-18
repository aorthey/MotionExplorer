#pragma once
#include "elements/swept_volume.h"
#include "elements/swath_volume.h"
#include "elements/simplicial_complex.h"
#include "planner/serialized_tree.h"

struct HierarchicalLevel{
  uint idx;
  std::string name;
  std::vector<Config> V;
  //geometry
  //vector<GLDraw::GeometryAppearance> geometry;
  SwathVolume swv;
};

class PlannerOutput{

  public:

    uint robot_idx;
    std::string name_robot;
    std::string name_algorithm;

    Config q_init;
    Config q_goal;
    Config dq_init;
    Config dq_goal;

    int drawSweptVolume;
    int drawMilestones;
    int drawStartGoal;
    int drawTree;
    int drawSimplicialComplex;

    SweptVolume *sv;
    SwathVolume *swv;
    SimplicialComplex cmplx;

    Robot *robot;
  private:
    double time;
    uint nodes;

    //tree swath
    SerializedTree _stree;

    //path
    std::vector<Config> q;
    std::vector<Config> dq;
    std::vector<Config> ddq;
    std::vector<Vector> torques;

    std::vector<HierarchicalLevel> hierarchy;

  public:

    PlannerOutput();

    SweptVolume& GetSweptVolume();
    SwathVolume& GetSwathVolume();

    void SetHierarchy(std::vector<HierarchicalLevel> &hierarchy_);
    const std::vector<HierarchicalLevel>& GetHierarchy();
    void SetTorques(std::vector<Vector> &torques_);
    const std::vector<Vector>& GetTorques();
    Config GetInitConfiguration();
    const SerializedTree& GetTree();
    void SetTree(SerializedTree &stree);
    const std::vector<Config> GetKeyframes();
    void SetKeyframes(std::vector<Config> &keyframes);
    const SimplicialComplex& GetSimplicialComplex();

};

