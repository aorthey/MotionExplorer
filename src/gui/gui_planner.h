#pragma once
#include "gui/gui.h"
#include "planner/planner.h"

class PlannerBackend : public ForceFieldBackend
{
  friend class GLUIPlannerGUI;

  protected:

    typedef ForceFieldBackend BaseT; 
    uint active_planner;
    std::vector<MotionPlanner*> planners;

    //play/replay/stop
    double t;

    PathPiecewiseLinear *path;

    SmartPointer<RobotController> controller_;

  public:

    PlannerBackend(RobotWorld *world);
    virtual void AddPlannerInput(PlannerMultiInput& _in);
    virtual void Start();
    virtual bool OnCommand(const string& cmd,const string& args);
    virtual void RenderWorld();
    virtual void RenderScreen();
    virtual void RenderCommand(const std::string &cmd);
    virtual bool OnIdle();
    std::string getRobotEnvironmentString();
    void CenterCameraOn(const Vector3& v);

    uint draw_cover_active_open_set{0};
    bool draw_cover_all_open_sets{true};
    int draw_path_modus{0};
};


class GLUIPlannerGUI: public GLUIForceFieldGUI
{
  public:
    typedef GLUIForceFieldGUI BaseT;
    GLUIPlannerGUI(GenericBackendBase* _backend,RobotWorld* _world);
    void AddPlannerInput(PlannerMultiInput& _in);
    virtual bool Initialize();
};


typedef SmartPointer<PlannerBackend> PlannerBackendPtr;
