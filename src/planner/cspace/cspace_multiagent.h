#pragma once
#include "planner/cspace/cspace.h"
#include "klampt.h"

namespace ob = ompl::base;
namespace oc = ompl::control;

class CSpaceOMPLMultiAgent: public CSpaceOMPL
{
  public:

    CSpaceOMPLMultiAgent(std::vector<CSpaceOMPL*> cspaces);

    virtual const oc::StatePropagatorPtr StatePropagatorPtr(oc::SpaceInformationPtr si) override;

    virtual void ConfigToOMPLState(const Config &q, ob::State *qompl) override;
    void ConfigToOMPLState(const Config &q, ob::State *qompl, int subspace);

    virtual Config OMPLStateToConfig(const ob::State *qompl) override;
    Config OMPLStateToConfig(const ob::State *qompl, int subspace);

    virtual uint GetKlamptDimensionality() const override;
    std::vector<int> GetKlamptDimensionalities() const;

    virtual bool SatisfiesBounds(const ob::State*) override;
    virtual bool UpdateRobotConfig(Config &q) override;

    // const ob::StateValidityCheckerPtr StateValidityCheckerPtr();
    // Vector3 getXYZ(const ob::State*);

    void setNextLevelRobotPointers(std::vector<int>);

    // void SetCSpaceInput(const CSpaceInput &input_);
    // CSpaceKlampt* GetCSpaceKlamptPtr();

    virtual void drawConfig(const Config &q, GLDraw::GLColor color=GLDraw::GLColor(1,0,0), double scale = 1.0) override;

    virtual bool IsPlanar() override;
    virtual bool isDynamic() const override;
    virtual bool isMultiAgent() const override;

    virtual void print(std::ostream& out) const override;

    std::vector<int> GetRobotIdxs() const;
    virtual Vector3 getXYZ(const ob::State*) override;
    virtual Vector3 getXYZ(const ob::State*, int) override;

    std::vector<Config> splitConfig(const Config &q);
  protected:

    virtual const ob::StateValidityCheckerPtr StateValidityCheckerPtr(ob::SpaceInformationPtr si) override;
    virtual void initSpace() override;

    std::vector<CSpaceOMPL*> cspaces_;

    std::vector<int> ptr_to_next_level_robot_ids;
    std::vector<int> robot_ids;

    std::vector<int> Nklampts;
    std::vector<int> Nompls;

    int subspaceCount{0};

};
