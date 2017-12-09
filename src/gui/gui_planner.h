#pragma once
#include "gui/gui.h"
#include "planner/planner.h"
#include "planner/planner_benchmark.h"

class PlannerBackend : public ForceFieldBackend
{
  friend class GLUIPlannerGUI;

  protected:

    typedef ForceFieldBackend BaseT; 
    uint active_planner;
    std::vector<MotionPlanner*> planners;

    //play/replay/stop
    double t;

  public:

    PlannerBackend(RobotWorld *world);
    virtual void AddPlannerInput(PlannerMultiInput& _in);
    virtual void Start();
    virtual bool OnCommand(const string& cmd,const string& args);
    virtual void RenderWorld();
    virtual void RenderScreen();
    virtual bool OnIdle();

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
