#pragma once
#include "gui.h"
#include "planner/planner.h"
#include "planner/planner_hierarchy.h"
#include "planner/planner_hierarchy_shallow.h"

class PlannerBackend : public ForceFieldBackend
{
  friend class GLUIPlannerGUI;

  protected:

    typedef ForceFieldBackend BaseT; 

    HierarchicalMotionPlanner *planner;

    std::vector<HierarchicalMotionPlanner*> planners;

    int active_planner;

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
    GLUIPlannerGUI(GenericBackendBase* _backend,RobotWorld* _world);
    void AddPlannerInput(PlannerInput& _in);
    virtual bool Initialize();

};


typedef SmartPointer<PlannerBackend> PlannerBackendPtr;
