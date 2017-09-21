#pragma once
#include "gui.h"
#include "planner/planner_hierarchy.h"

class PlannerBackend : public ForceFieldBackend
{
  friend class GLUIPlannerGUI;

  protected:

    typedef ForceFieldBackend BaseT; //Need to parse it through SimTest to get wrenchies

    PlannerInput in;

    HierarchicalMotionPlanner *planner;

  public:

    PlannerBackend(RobotWorld *world);
    virtual void AddPlannerInput(PlannerInput& _in);
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
    GLUIPlannerGUI(GenericBackendBase* _backend,RobotWorld* _world, PlannerInput _in);
    virtual bool Initialize();
    // virtual void Handle_Keypress(unsigned char c,int x,int y);
    // virtual bool OnCommand(const string& cmd,const string& args);
    // virtual void Handle_Control(int id);

  private:
    PlannerInput in;
};


typedef SmartPointer<PlannerBackend> PlannerBackendPtr;
