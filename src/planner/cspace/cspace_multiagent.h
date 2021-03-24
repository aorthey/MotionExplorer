#pragma once
#include "planner/cspace/cspace.h"
#include "planner/cspace/cspace_kinodynamic.h"
#include "klampt.h"

namespace ob = ompl::base;
namespace oc = ompl::control;

class CSpaceOMPLMultiAgent: public CSpaceOMPL
{
    using BaseT = CSpaceOMPL;
    using ConfigVelocity = std::pair<Config, Config>;
  public:

    CSpaceOMPLMultiAgent(std::vector<CSpaceOMPL*> cspaces);

    virtual void ConfigToOMPLState(const Config &q, ob::State *qompl) override;
    void ConfigToOMPLState(const Config &q, ob::State *qompl, int subspace);

    virtual void ConfigVelocityToOMPLState(const Config &q, const Config &dq, ob::State *qompl);
    virtual ob::ScopedState<> ConfigVelocityToOMPLState(const Config &q, const Config &dq);
    void ConfigVelocityToOMPLState(const Config &q, const Config &dq, ob::State *qompl, int subspace);

    virtual Config OMPLStateToConfig(const ob::State *qompl) override;
    Config OMPLStateToConfig(const ob::State *qompl, int subspace);
    virtual Config OMPLStateToVelocity(const ob::State *qompl) override;
    Config OMPLStateToVelocity(const ob::State *qompl, int subspace);

    virtual uint GetKlamptDimensionality() const override;
    std::vector<int> GetKlamptDimensionalities() const;

    virtual bool SatisfiesBounds(const ob::State*) override;
    virtual bool UpdateRobotConfig(Config &q) override;

    void setNextLevelRobotPointers(std::vector<int>);

    virtual void drawConfig(const Config &q, GLDraw::GLColor color=GLDraw::GLColor(1,0,0), double scale = 1.0) override;

    virtual bool IsPlanar() override;
    virtual bool isDynamic() const override;
    virtual bool isMultiAgent() const override;

    virtual ob::SpaceInformationPtr SpaceInformationPtr() override;

    virtual void print(std::ostream& out) const override;
    virtual bool isTimeDependent() override;

    std::vector<int> GetRobotIdxs() const;
    std::vector<int> GetProjectionIdxs() const;
    virtual Vector3 getXYZ(const ob::State*) override;
    virtual Vector3 getXYZ(const ob::State*, int) override;

    std::vector<Config> splitConfig(const Config &q);
    std::vector<ConfigVelocity> splitConfig(const Config &q, const Config &dq);

    virtual void initSpace() override;
    virtual void initControlSpace();

    virtual void SetTime(ob::State *qompl, double time) override;
    virtual double GetTime(const ob::State *qompl) override;
    virtual void DrawGL(GUIState& state) override;
  protected:

    virtual const ob::StateValidityCheckerPtr StateValidityCheckerPtr(ob::SpaceInformationPtr si) override;
    std::vector<CSpaceOMPL*> cspaces_;

    std::vector<int> ptr_to_next_level_robot_ids;
    std::vector<int> robot_ids;

    int idxTimeSpace_{-1};

    std::vector<int> Nklampts;
    std::vector<int> Nompls;

    int subspaceCount{0};

    ob::StateSpacePtr emptySpace_{nullptr};

};
