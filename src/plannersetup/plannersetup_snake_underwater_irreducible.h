#pragma once
#include "plannersetup_snake_underwater.h"

class PlannerSetupSnakeUnderwaterIrreducible: public PlannerSetupSnakeUnderwater
{

  public:
    PlannerSetupSnakeUnderwaterIrreducible(RobotWorld* world):
      PlannerSetupSnakeUnderwater(world){
    }

    virtual std::string GetSimulationFileString(){
      return std::string("/home/aorthey/git/orthoklampt/data/snake_underwater_irreducible.xml");
    }
};


