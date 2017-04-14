#pragma once

class PlannerSetup{
  private:
    RobotWorld *_world;
    Config qinit, qgoal;

  public:
    PlannerSetup(RobotWorld* world){
      _world = world;
    }
    void LoadAndInitSim(ForceFieldBackend& backend){
      std::string file = this->GetSimulationFileString();
      backend.LoadAndInitSim(file.c_str());
      Robot *robot = _world->robots[0];
      qinit = GetInitialConfigSetup(robot);
      qgoal = GetGoalConfigSetup(robot);
      SetRobotSE3Limits(robot);
    }

    Config GetInitialConfig(){ return qinit; }
    Config GetGoalConfig(){ return qgoal; }

  protected:
    virtual std::string GetSimulationFileString() = 0;
    virtual Config GetInitialConfigSetup(Robot *robot) = 0;
    virtual Config GetGoalConfigSetup(Robot *robot) = 0;
    virtual void SetRobotSE3Limits(Robot *robot) = 0;

};

