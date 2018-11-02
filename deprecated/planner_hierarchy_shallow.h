#pragma once

#include "planner/planner.h"

//struct SinglePathNode{
//    std::vector<Config> path;
//    std::vector<ob::State*> state_path;
//    bool isSufficient;
//
//    SinglePathNode(){ sv = NULL; }
//    SweptVolume& GetSweptVolume(Robot *robot){
//      if(!sv){
//        sv = new SweptVolume(robot, path, 0);
//      }
//      return *sv;
//    }
//    SweptVolume *sv;
//};
class ShallowMotionPlanner: public MotionPlanner<PathSpace>{

  public:
    ShallowMotionPlanner(RobotWorld *world_, PlannerInput& input_);

    virtual void Expand();
    virtual void Collapse();
    virtual void Next();
    virtual void Previous();
    virtual void DrawGLScreen(double x_ =0.0, double y_=0.0);
    virtual void DrawGL(const GUIState&);

  private:
    std::vector<Config> solution_path;

};

