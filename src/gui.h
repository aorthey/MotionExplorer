#pragma once

#include <Interface/SimTestGUI.h>
#include <Modeling/MultiPath.h>
#include <KrisLibrary/robotics/IK.h>
#include <KrisLibrary/robotics/IKFunctions.h>
#include <KrisLibrary/geometry/CollisionMesh.h>
#include <KrisLibrary/planning/KinodynamicPath.h>
#include <KrisLibrary/planning/KinodynamicMotionPlanner.h>
#include <KrisLibrary/GLdraw/drawMesh.h>
#include <KrisLibrary/GLdraw/GLError.h>

#include <View/ViewIK.h>
#include <ode/ode.h>
#include "planner.h"

#define DEBUG 0

class ForceFieldBackend : public SimTestBackend
{
  private:
    bool drawForceField;
    bool drawRobotExtras; 
    bool drawIKextras;
    bool drawPath;
    bool drawPlannerTree;
    bool drawPlannerStartGoal;
    bool drawAxesLabels;
    vector<IKGoal> _constraints;
    vector<int> _linksInCollision;
    string _robotname;
    SerializedTree _stree;
    //swept volume
    Config planner_p_init, planner_p_goal;
    std::vector<std::vector<Matrix4> > _mats;
    vector<GLDraw::GeometryAppearance> _appearanceStack;
    typedef SimTestBackend BaseT; //Need to parse it through SimTest to get wrenchies

  public:

  ForceFieldBackend(RobotWorld *world);
  virtual void Start();
  virtual bool OnCommand(const string& cmd,const string& args);
  void SetIKConstraints( vector<IKGoal> constraints, string robotname);
  void SetIKCollisions( vector<int> linksInCollision );
  void VisualizePathSweptVolumeAtPosition(const Config &q);
  void VisualizePathSweptVolume(const MultiPath &path);
  void VisualizePathSweptVolume(const KinodynamicMilestonePath &path);
  void VisualizePlannerTree(const SerializedTree &tree);
  void VisualizeStartGoal(const Config &p_init, const Config &p_goal);
  virtual void RenderWorld();


};

