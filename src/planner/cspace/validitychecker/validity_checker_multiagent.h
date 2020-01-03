#pragma once
#include "planner/cspace/cspace.h"
#include "neighborhood.h"

class OMPLValidityCheckerMultiAgent: public ob::StateValidityChecker
{
  public:
    OMPLValidityCheckerMultiAgent(const ob::SpaceInformationPtr &si, std::vector<CSpaceOMPL*> cspaces_);

    // bool IsFeasible(const ob::State* state) const;

    bool isValid(const ob::State* state) const override;
    // bool IsCollisionFree(SingleRobotCSpace *space, Config q) const;

  protected:
    std::vector<CSpaceOMPL*> cspaces_;
    std::vector<SingleRobotCSpace*> klampt_single_robot_cspaces_;
};

typedef std::shared_ptr<OMPLValidityCheckerMultiAgent> OMPLValidityCheckerMultiAgentPtr;
