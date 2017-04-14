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

class SweptVolume
{
  public:
    SweptVolume(Robot *robot){
      _keyframes.clear();
      _mats.clear();
      _robot = robot;
      color = GLColor(0.8,0.8,0.8);
    }

    SweptVolume(Robot *robot, const std::vector<Config> &keyframes, uint Nkeyframes)
    {
      _keyframes.clear();
      _mats.clear();
      _robot = robot;
      color = GLColor(0.8,0.8,0.8);

      for(int i = 0; i < keyframes.size(); i++)
      {
        Config q = keyframes.at(i);
        AddKeyframe(q);
      }
      init = _keyframes.front();
      goal = _keyframes.back();
      //compute keyframe indices 
      if(Nkeyframes > keyframes.size()){
        Nkeyframes = keyframes.size();
      }
      if(Nkeyframes < 1){
        Nkeyframes=0;
      }
      _keyframe_indices.clear();

      uint N = keyframes.size();
      uint Nstep = (int)(N/Nkeyframes);

      if(Nstep<1) Nstep=1;
      uint Ncur = 0;
      while(Ncur < N){
        _keyframe_indices.push_back(Ncur);
        Ncur += Nstep;
      }
      std::cout << "Milestone visualization indicies: " << _keyframe_indices << std::endl;

    }

    const std::vector<std::vector<Matrix4> >& GetMatrices(){
      return _mats;
    }
    const std::vector<Config >& GetKeyframes(){
      return _keyframes;
    }
    void SetColor(const GLColor c){
      color = c;
    }
    GLColor GetColor(){
      return color;
    }
    void SetColorMilestones(const GLColor c){
      color_milestones= c;
    }
    GLColor GetColorMilestones(){
      return color_milestones;
    }
    const Config& GetStart(){
      return init;
    }
    const Config& GetGoal(){
      return goal;
    }
    const vector<uint>& GetKeyframeIndices(){
      return _keyframe_indices;
    }
  private:
    void AddKeyframe(const Config &q ){

      if(!_robot->InJointLimits(q)){
        std::cout << "trying to set an outer limit config" << std::endl;
        std::cout << "minimum       :" << _robot->qMin << std::endl;
        std::cout << "configuration :" << q << std::endl;
        std::cout << "maximum       :" << _robot->qMax << std::endl;
        exit(0);
      }
      _robot->UpdateConfig(q);

      std::vector<Matrix4> mats_config;
      for(size_t i=0;i<_robot->links.size();i++) {
        Matrix4 mat = _robot->links[i].T_World;
        mats_config.push_back(mat);
      }
      _mats.push_back(mats_config);
      _keyframes.push_back(q);
    }

    GLColor color;
    GLColor color_milestones;
    Robot *_robot;
    std::vector<std::vector<Matrix4> > _mats;
    vector<Config> _keyframes;
    Config init, goal;
    vector<uint> _keyframe_indices;

};

class ForceFieldBackend : public SimTestBackend
{
  private:
    typedef SimTestBackend BaseT; //Need to parse it through SimTest to get wrenchies

    vector<IKGoal> _constraints;
    vector<int> _linksInCollision;
    string _robotname;
    SerializedTree _stree;
    vector<GLDraw::GeometryAppearance> _appearanceStack;

    std::vector<SweptVolume> swept_volume_paths;

    //swept volume
    Config planner_p_init, planner_p_goal;
    std::vector<std::vector<Matrix4> > _mats;
    vector<Config> _keyframes;
    vector<uint> _milestonekeyframe_indices;

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


  //void ShowPath(){ drawPath=true; }
  //void HidePath(){ drawPath=false; }
  //void ShowPlannerTree(){ drawPlannerTree=true; }
  //void HidePlannerTree(){ drawPlannerTree=false; }
  //void ShowPlannerStartGoal(){ drawPlannerStartGoal=true; }
  //void HidePlannerStartGoal(){ drawPlannerStartGoal=false; }

  void ShowCoordinateAxes(){ drawAxes = 1; }
  void HideCoordinateAxes(){ drawAxes = 0; }
  void ShowRobot(){ drawRobot = 1; }
  void HideRobot(){ drawRobot = 0; }

  void VisualizeFrame( const Vector3 &p, const Vector3 &e1, const Vector3 &e2, const Vector3 &e3, double frameLength=1.0);
  void VisualizeStartGoal(const Config &p_init, const Config &p_goal);
  void SetIKConstraints( vector<IKGoal> constraints, string robotname);
  void SetIKCollisions( vector<int> linksInCollision );

  uint getNumberOfPaths();

  void AddPath(const std::vector<Config> &keyframes, GLColor color = GLColor(0.8,0.8,0.8), uint Nkeyframes_alongpath=10);
  //deprecated
  //void VisualizePathSweptVolumeAtPosition(const Config &q);
  //void VisualizePathSweptVolume(const std::vector<Config> &keyframes);
  //void VisualizePathSweptVolume(const MultiPath &path);
  //void VisualizePathSweptVolume(const KinodynamicMilestonePath &path);
  //void VisualizeStartGoal(const Config &p_init, const Config &p_goal);
  //void VisualizePathMilestones(const std::vector<Config> &keyframes, uint Nmilestones);

  void VisualizePlannerTree(const SerializedTree &tree);
  std::vector<Config> getKeyFrames();

  std::vector<int> drawPathSweptVolume;
  std::vector<int> drawPathMilestones;
  std::vector<int> drawPathStartGoal;

  int drawForceField;
  int drawRobotExtras; 
  int drawIKextras;
  int drawRobot;
  int drawPlannerTree;
  int drawPlannerStartGoal;
  int drawAxes;
  int drawAxesLabels;
  int drawRigidObjects;
  int drawRigidObjectsEdges;
  int drawRigidObjectsFaces;
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

