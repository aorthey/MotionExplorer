#pragma once
#include "planner/cspace/cspace_geometric.h"

class CSpaceOMPLTime: public CSpaceOMPL
{
  public:
    typedef std::shared_ptr<PathPiecewiseLinear> PathPiecewiseLinearPtr;

    CSpaceOMPLTime(RobotWorld *world_);
    virtual bool isDynamic() const override;
    virtual void initSpace();
    virtual void ConfigToOMPLState(const Config &q, ob::State *qompl) override;
    virtual Config OMPLStateToConfig(const ob::State *qompl);
    virtual void print(std::ostream& out = std::cout) const;
    virtual double GetTime(const ob::State *qompl) override;
    virtual void SetTime(ob::State *qompl, double time) override;
    virtual Vector3 getXYZ(const ob::State*) override;

    virtual uint GetKlamptDimensionality() const override;
    // void UpdateEnvironmentFromState(const ob::State*);

  protected:
    //information about external dynamically moving robots
    std::vector<int> externalRobotsIdxs;
    std::vector<std::string> externalRobotsPathFiles;
};

