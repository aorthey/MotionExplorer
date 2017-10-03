#pragma once
#include "planner/serialized_tree.h"
#include "planner/planner_output.h"
#include "elements/swept_volume.h"
#include "elements/wrench_field.h"
#include "controller/controller.h"
#include "gui_state.h"

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

#define DEBUG 0


class ForceFieldBackend : public SimTestBackend
{
    friend class GLUIForceFieldGUI;
  protected:

    typedef SimTestBackend BaseT; //Need to parse it through SimTest to get wrenchies

    vector<IKGoal> _constraints;
    vector<int> _linksInCollision;
    string _robotname;
    //##########################################################################
    vector<GLDraw::GeometryAppearance> _appearanceStack;
    //##########################################################################
    vector< vector<Vector3> > _frames;
    vector< double > _frameLength;

    vector<PlannerOutput> plannerOutput;

  public:

    WrenchField wrenchfield;

    ForceFieldBackend(RobotWorld *world);
    //virtual ~ForceFieldBackend(){};
    virtual void Start();
    virtual bool OnCommand(const string& cmd,const string& args);
    virtual void RenderWorld();
    virtual void RenderScreen();

    virtual bool OnIdle();
    void SendPlannerOutputToController();
    void SendCommandStringController(string cmd, string arg);

    virtual bool Save(const char* file=NULL);
    virtual bool Save(TiXmlElement *node);
    virtual bool Load(const char* file);
    virtual bool Load(TiXmlElement *node);

    void AddPlannerOutput( PlannerOutput pout );

    void VisualizeFrame( const Vector3 &p, const Vector3 &e1, const Vector3 &e2, const Vector3 &e3, double frameLength=1.0);
    void VisualizeStartGoal(const Config &p_init, const Config &p_goal);
    void SetIKConstraints( vector<IKGoal> constraints, string robotname);
    void SetIKCollisions( vector<int> linksInCollision );

    uint getNumberOfPaths();

    void VisualizePlannerTree(const SerializedTree &tree);

    //void DrawText(int x,int y, std::string s);
    void DrawText(int x,int y, std::string s, void* font = GLUT_BITMAP_HELVETICA_18);
    void DrawTextVector(double xpos, double ypos, const char* prefix, Vector &v);

    std::vector<int> showLinks; //hide certain links 

    std::vector<int> drawPathShortestPath;
    std::vector<int> drawPathSweptVolume;
    std::vector<int> drawPathMilestones;
    std::vector<int> drawPathStartGoal;
    std::vector<int> drawPlannerTree;
    std::vector<int> drawPlannerSimplicialComplex;

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
    int drawAxes;
    int drawAxesLabels;

    void toggle(int &k){
      if(k) k=0;
      else k=1;
    }

    int line_x_pos;
    int line_y_offset;
    int line_y_offset_stepsize;

    GUIState state;

};


class GLUIForceFieldGUI: public GLUISimTestGUI
{
  public:
    typedef GLUISimTestGUI BaseT;
    GLUIForceFieldGUI(GenericBackendBase* _backend,RobotWorld* _world);
    virtual bool Initialize();
    //virtual bool OnCommand(const string& cmd,const string& args);
    void AddToKeymap(const char *key, const char *s, bool baseClass = false);
    void AddButton(const char *key);
    void browser_control(int control);

    virtual void Handle_Keypress(unsigned char c,int x,int y);
    virtual bool OnCommand(const string& cmd,const string& args);
    virtual void Handle_Control(int id);

  private:
    typedef std::map<const char *, std::string> Keymap;
    Keymap _keymap;
    Keymap _baseclass_keys;
    GLUI_Panel* panel;
    GLUI_Checkbox* checkbox;
    GLUI_FileBrowser *browser;
    GLUI_Button* button_file_load;
    GLUI_Listbox* linkBox;

};


typedef SmartPointer<ForceFieldBackend> ForceFieldBackendPtr;
