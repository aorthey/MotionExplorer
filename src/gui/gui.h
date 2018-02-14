#pragma once
#include "planner/planner_output.h"
#include "elements/swept_volume.h"
#include "elements/wrench_field.h"
#include "controller/controller.h"
#include "gui/gui_state.h"

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

    //void VisualizeFrame( const Vector3 &p, const Vector3 &e1, const Vector3 &e2, const Vector3 &e3, double frameLength=1.0);
    void DrawText(int x,int y, std::string s, void* font = GLUT_BITMAP_HELVETICA_18);
    void DrawTextVector(double xpos, double ypos, const char* prefix, Vector &v);

    int line_x_pos;
    int line_y_offset;
    int line_y_offset_stepsize;

    uint active_robot;
    GUIState state;
};


class GLUIForceFieldGUI: public GLUISimTestGUI
{
  public:
    typedef GLUISimTestGUI BaseT;
    GLUIForceFieldGUI(GenericBackendBase* _backend,RobotWorld* _world);
    virtual bool Initialize();
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
    GLUI_Button* button;
    GLUI_Listbox* linkBox;
    GLUI_Spinner* spinner;
};


typedef SmartPointer<ForceFieldBackend> ForceFieldBackendPtr;
