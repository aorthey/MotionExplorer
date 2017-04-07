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
#include "planner/serialized_tree.h"

#define DEBUG 0

class ForceFieldBackend : public SimTestBackend
{
  private:
    int drawForceField;
    int drawRobotExtras; 
    int drawIKextras;
    int drawPath;
    int drawPathMilestones;
    int drawPlannerTree;
    int drawPlannerStartGoal;
    int drawAxes;
    int drawAxesLabels;
    int drawRigidObjects;
    int drawRigidObjectsEdges;
    int drawRigidObjectsFaces;

    vector<IKGoal> _constraints;
    vector<int> _linksInCollision;
    string _robotname;
    SerializedTree _stree;
    //swept volume
    Config planner_p_init, planner_p_goal;
    std::vector<std::vector<Matrix4> > _mats;
    vector<GLDraw::GeometryAppearance> _appearanceStack;
    vector<Config> _keyframes;
    vector<uint> _milestonekeyframe_indices;
    typedef SimTestBackend BaseT; //Need to parse it through SimTest to get wrenchies

    vector< vector<Vector3> > _frames;
    vector< double > _frameLength;

  public:

  ForceFieldBackend(RobotWorld *world);
  virtual void Start();
  virtual bool OnCommand(const string& cmd,const string& args);
  virtual void RenderWorld();

  virtual bool Save(const char* file=NULL);
  virtual bool Save(TiXmlElement *node);
  virtual bool Load(const char* file);
  virtual bool Load(TiXmlElement *node);

  void ShowPath(){ drawPath=true; }
  void HidePath(){ drawPath=false; }
  void ShowPlannerTree(){ drawPlannerTree=true; }
  void HidePlannerTree(){ drawPlannerTree=false; }
  void ShowPlannerStartGoal(){ drawPlannerStartGoal=true; }
  void HidePlannerStartGoal(){ drawPlannerStartGoal=false; }

  void VisualizeFrame( const Vector3 &p, const Vector3 &e1, const Vector3 &e2, const Vector3 &e3, double frameLength=1.0);
  void SetIKConstraints( vector<IKGoal> constraints, string robotname);
  void SetIKCollisions( vector<int> linksInCollision );
  void VisualizePathSweptVolumeAtPosition(const Config &q);
  void VisualizePathSweptVolume(const MultiPath &path);
  void VisualizePathSweptVolume(const KinodynamicMilestonePath &path);
  void VisualizePathSweptVolume(const std::vector<Config> &keyframes);
  std::vector<Config> getKeyFrames();
  void VisualizePlannerTree(const SerializedTree &tree);
  void VisualizeStartGoal(const Config &p_init, const Config &p_goal);
  void VisualizePathMilestones(const std::vector<Config> &keyframes, uint Nmilestones);
};


class GLUIForceFieldGUI: public GLUISimTestGUI
{
  public:
    typedef GLUISimTestGUI BaseT;
    GLUIForceFieldGUI(GenericBackendBase* _backend,RobotWorld* _world);
    virtual bool Initialize();
    virtual bool OnCommand(const string& cmd,const string& args);
    virtual void Handle_Keypress(unsigned char c,int x,int y);

    void browser_control(int control);
  private:
    GLUI_Panel* panel;
    GLUI_Checkbox* checkbox;
    GLUI_FileBrowser *browser;
};

