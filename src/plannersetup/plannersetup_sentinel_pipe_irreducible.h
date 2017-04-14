#pragma once
#include "plannersetup_sentinel_pipe.h"

class PlannerSetupSentinelPipeIrreducible: public PlannerSetupSentinelPipe
{

  public:
    PlannerSetupSentinelPipeIrreducible(RobotWorld* world):
      PlannerSetupSentinelPipe(world){
    }

    virtual std::string GetSimulationFileString(){
      return std::string("/home/aorthey/git/orthoklampt/data/sentinel_pipedreamin.xml");
    }

};


