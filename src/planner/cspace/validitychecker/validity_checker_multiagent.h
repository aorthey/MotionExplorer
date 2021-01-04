#pragma once
#include "planner/cspace/cspace.h"
#include "planner/cspace/cspace_multiagent.h"
#include "neighborhood.h"
#include <ompl/base/StateValidityCheckerDifferentiable.h>

class OMPLValidityCheckerMultiAgent: public ob::StateValidityCheckerDifferentiable
{
  public:
    OMPLValidityCheckerMultiAgent(const ob::SpaceInformationPtr &si, CSpaceOMPLMultiAgent *cspace, std::vector<CSpaceOMPL*> cspaces_);

    bool isValid(const ob::State* state) const override;

    virtual double clearance(const ob::State*) const override;

  protected:
    double DistanceToConstraints(const ob::State* state) const;

    CSpaceOMPLMultiAgent *cspace_;
    std::vector<CSpaceOMPL*> cspaces_;
    std::vector<SingleRobotCSpace*> klampt_single_robot_cspaces_;

    int idxTimeSpace_{-1};
};

typedef std::shared_ptr<OMPLValidityCheckerMultiAgent> OMPLValidityCheckerMultiAgentPtr;
