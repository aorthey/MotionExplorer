#pragma once
#include "plannersetup.h"

class PlannerSetupSnakeUnderwater: public PlannerSetup
{

  public:
    PlannerSetupSnakeUnderwater(RobotWorld* world):
      PlannerSetup(world){
    }

    virtual std::string GetSimulationFileString(){
      return std::string("/home/aorthey/git/orthoklampt/data/snake_underwater.xml");
    }

    virtual Config GetInitialConfigSetup(Robot *robot){
      Config p_init = robot->q;
      p_init.setZero();

      p_init[0]=0.0;
      p_init[1]=0.0;
      p_init[2]=3;
      p_init[3]=M_PI/4;
      p_init[4]=M_PI/4;
      p_init[5]=0;
      return p_init;
    }

    virtual Config GetGoalConfigSetup(Robot *robot){
      Config p_goal = robot->q;
      p_goal.setZero();

      p_goal[0]=-3.5;
      p_goal[1]=2.8;
      p_goal[2]=0.4;
      p_goal[3]=M_PI-M_PI/32;
      return p_goal;
    }

    virtual void SetRobotSE3Limits(Robot *robot){
      robot->qMin[0]=-4;
      robot->qMin[1]=-4;
      robot->qMin[2]=-1;
      robot->qMin[3]=-M_PI;
      robot->qMin[4]=-M_PI/2;
      robot->qMin[5]=-M_PI;

      robot->qMax[0]=4;
      robot->qMax[1]=4;
      robot->qMax[2]=4;
      robot->qMax[3]=M_PI;
      robot->qMax[4]=M_PI/2;
      robot->qMax[5]=M_PI;
    }

};


