#pragma once
#include "planner/cspace/cspace.h"
#include "planner/cspace/cspace_multiagent.h"
#include "neighborhood.h"

class OMPLValidityCheckerMultiAgent: public ob::StateValidityChecker
{
  public:
    OMPLValidityCheckerMultiAgent(const ob::SpaceInformationPtr &si, CSpaceOMPLMultiAgent *cspace, std::vector<CSpaceOMPL*> cspaces_);

    // bool IsFeasible(const ob::State* state) const;

    bool isValid(const ob::State* state) const override;
    // bool IsCollisionFree(SingleRobotCSpace *space, Config q) const;

  protected:
    CSpaceOMPLMultiAgent *cspace_;

    std::vector<CSpaceOMPL*> cspaces_;
    std::vector<SingleRobotCSpace*> klampt_single_robot_cspaces_;
};

typedef std::shared_ptr<OMPLValidityCheckerMultiAgent> OMPLValidityCheckerMultiAgentPtr;
