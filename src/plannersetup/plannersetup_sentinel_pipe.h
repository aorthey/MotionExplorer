#pragma once
#include "plannersetup.h"

class PlannerSetupSentinelPipe: public PlannerSetup
{

  public:
    PlannerSetupSentinelPipe(RobotWorld* world):
      PlannerSetup(world){
    }

    virtual std::string GetSimulationFileString(){
      return std::string("/home/aorthey/git/orthoklampt/data/sentinel_pipedreamin_complete.xml");
    }

    virtual Config GetInitialConfigSetup(Robot *robot){
      Config p_init = robot->q;
      p_init.setZero();

      p_init[0]=-2.5;
      p_init[1]=-1.5;
      p_init[2]=4.2;
      p_init[3]=-M_PI/4;
      return p_init;
    }

    virtual Config GetGoalConfigSetup(Robot *robot){
      Config p_goal = robot->q;
      p_goal.setZero();

      ////goal in lower pipe
      //p_goal[0]=-0.5;
      //p_goal[1]=1.0;
      //p_goal[2]=0.0;
      //p_goal[3]=M_PI/2;

      //////goal in upper pipe
      p_goal[0]=-5.1;
      p_goal[1]=-0.1;
      p_goal[2]=15;
      p_goal[3]=0;
      p_goal[4]=-M_PI/2;
      p_goal[5]=0;
      return p_goal;
    }

    virtual void SetRobotSE3Limits(Robot *robot){
      robot->qMin[0]=-6;
      robot->qMin[1]=-6;
      robot->qMin[2]=-1;
      robot->qMin[3]=-M_PI;
      robot->qMin[4]=-M_PI/2;
      robot->qMin[5]=-M_PI;

      robot->qMax[0]=6;
      robot->qMax[1]=6;
      robot->qMax[2]=16;
      robot->qMax[3]=M_PI;
      robot->qMax[4]=M_PI/2;
      robot->qMax[5]=M_PI;
    }

};


