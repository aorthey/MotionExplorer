#pragma once
#include "planner/cspace/cspace.h"

class OMPLValidityChecker: public ob::StateValidityChecker
{
  public:
    OMPLValidityChecker(const ob::SpaceInformationPtr &si, CSpaceOMPL *cspace_);

    virtual bool IsSufficientFeasible(const ob::State* state) const;
    virtual bool IsFeasible(const ob::State* state) const;

    double Distance(const ob::State* state) const;
    bool isValid(const ob::State* state) const override;
    bool IsCollisionFree(SingleRobotCSpace *space, Config q) const;

    CSpaceOMPL* GetCSpaceOMPLPtr() const;

  protected:
    CSpaceOMPL *cspace;
    SingleRobotCSpace *klampt_single_robot_cspace;
};

// class OMPLValidityCheckerInnerOuter: public OMPLValidityChecker
// {
//   public:
//     OMPLValidityCheckerInnerOuter(const ob::SpaceInformationPtr &si, CSpaceOMPL *ompl_space_, CSpace *inner_, CSpace *outer_);
//     virtual bool isValid(const ob::State* state) const;

//     CSpace *outer;
// };

class OMPLValidityCheckerNecessarySufficient: public OMPLValidityChecker
{
  public:
    OMPLValidityCheckerNecessarySufficient(const ob::SpaceInformationPtr &si, CSpaceOMPL *ompl_space_, CSpaceKlampt *outer_);
    virtual bool IsSufficientFeasible(const ob::State* state) const;

    SingleRobotCSpace *klampt_single_robot_cspace_outer_approximation;
};
typedef std::shared_ptr<OMPLValidityChecker> OMPLValidityCheckerPtr;
