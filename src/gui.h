#pragma once

#include <regex>
#include <Interface/SimTestGUI.h>
#include <Modeling/MultiPath.h>
#include <KrisLibrary/robotics/IK.h>
#include <KrisLibrary/robotics/IKFunctions.h>
#include <KrisLibrary/geometry/CollisionMesh.h>
#include <KrisLibrary/geometry/AnyGeometry.h>
#include <KrisLibrary/planning/KinodynamicPath.h>
#include <KrisLibrary/planning/KinodynamicMotionPlanner.h>
#include <KrisLibrary/GLdraw/drawMesh.h>
#include <KrisLibrary/GLdraw/GLError.h>
#include <KrisLibrary/GLdraw/GLUTString.h>

#include <View/ViewIK.h>
#include <ode/ode.h>
#include "planner/serialized_tree.h"
#include "elements/swept_volume.h"
#include "elements/wrench_field.h"
#include "controller/controller.h"

#define DEBUG 0

class GUIVariable
{
  private:
    bool active;
    char hotkey;
    std::string name;
    std::string descr;
  public:

    GUIVariable(std::string name);
    GUIVariable(std::string name, std::string descr);
    GUIVariable(std::string name, bool active);
    GUIVariable(std::string name, char hotkey);
    GUIVariable(std::string name, std::string descr, char hotkey);
    GUIVariable(std::string name, std::string descr, char hotkey, bool active);

    void toggle(){
      if(active) active=false;
      else active=true;
    }

    operator bool() const{
      return active;
    }
    bool operator!() const{
      return !active;
    }
};

class ForceFieldBackend : public SimTestBackend
{
  private:
    typedef SimTestBackend BaseT; //Need to parse it through SimTest to get wrenchies

    vector<IKGoal> _constraints;
    vector<int> _linksInCollision;
    string _robotname;
    //##########################################################################
    SerializedTree _stree;
    vector<GLDraw::GeometryAppearance> _appearanceStack;

    //##########################################################################
    std::vector<SweptVolume> swept_volume_paths;

    Config planner_p_init, planner_p_goal;

    //##########################################################################
    vector< vector<Vector3> > _frames;
    vector< double > _frameLength;

    //point and direction of wrench for each link's COM

    //typedef Vector Wrench;
    //typedef vector< std::pair<Vector3, Wrench> > WrenchField;

  public:

  WrenchField wrenchfield;


  ForceFieldBackend(RobotWorld *world);
  virtual void Start();
  virtual bool OnCommand(const string& cmd,const string& args);
  virtual void RenderWorld();
  virtual void RenderScreen();

  virtual bool OnIdle();

  virtual bool Save(const char* file=NULL);
  virtual bool Save(TiXmlElement *node);
  virtual bool Load(const char* file);
  virtual bool Load(TiXmlElement *node);

  void ShowCoordinateAxes(){ drawAxes = 1; }
  void HideCoordinateAxes(){ drawAxes = 0; }
  void ShowRobot(){ drawRobot = 1; }
  void HideRobot(){ drawRobot = 0; }
  void ShowPlannerTree(){ drawPlannerTree = 1; }
  void HidePlannerTree(){ drawPlannerTree = 0; }
  void ShowSweptVolumes(){ showSweptVolumes = 1; }
  void HideSweptVolumes(){ showSweptVolumes = 0; }

  void VisualizeFrame( const Vector3 &p, const Vector3 &e1, const Vector3 &e2, const Vector3 &e3, double frameLength=1.0);
  void VisualizeStartGoal(const Config &p_init, const Config &p_goal);
  void SetIKConstraints( vector<IKGoal> constraints, string robotname);
  void SetIKCollisions( vector<int> linksInCollision );

  uint getNumberOfPaths();

  void AddPath(const std::vector<Config> &keyframes, GLColor color = GLColor(0.8,0.8,0.8), uint Nkeyframes_alongpath=10);
  void AddPathInterpolate(const std::vector<Config> &keyframes, GLColor color = GLColor(0.8,0.8,0.8), uint Nkeyframes_alongpath=10);

  const std::vector<Config>& getPathKeyFrames(uint pathid);
  void ClearPaths();
  void VisualizePlannerTree(const SerializedTree &tree);

  void DrawText(int x,int y, std::string s);

  std::vector<int> showLinks; //hide certain links 
  std::vector<int> drawPathSweptVolume;
  std::vector<int> drawPathMilestones;
  std::vector<int> drawPathStartGoal;

  int showSweptVolumes;

  int drawController;
  int drawContactDistances;

  int drawForceEllipsoid;
  int drawDistanceRobotTerrain;
  int drawCenterOfMassPath;

  int drawForceField;
  int drawWrenchField;
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

  void toggle(int &k){
    if(k) k=0;
    else k=1;
  }
};


class GLUIForceFieldGUI: public GLUISimTestGUI
{
  public:
    typedef GLUISimTestGUI BaseT;
    GLUIForceFieldGUI(GenericBackendBase* _backend,RobotWorld* _world);
    virtual bool Initialize();
    //virtual bool OnCommand(const string& cmd,const string& args);
    void AddToKeymap(const char *key, const char *s);
    void AddButton(const char *key);
    void browser_control(int control);

    virtual void Handle_Keypress(unsigned char c,int x,int y);
    virtual bool OnCommand(const string& cmd,const string& args);
    virtual void Handle_Control(int id);

  private:
    typedef std::map<const char *, std::string> Keymap;
    Keymap _keymap;
    GLUI_Panel* panel;
    GLUI_Checkbox* checkbox;
    GLUI_FileBrowser *browser;
    GLUI_Button* button_file_load;
    GLUI_Listbox* linkBox;

};


typedef SmartPointer<ForceFieldBackend> ForceFieldBackendPtr;

