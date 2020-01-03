#include "planner/cspace/cspace_geometric.h"

class GeometricCSpaceOMPLRNTime: public GeometricCSpaceOMPL
{
  public:
    GeometricCSpaceOMPLRNTime(RobotWorld *world_, int robot_id, int dimension);
    virtual void initSpace();
    virtual void ConfigToOMPLState(const Config &q, ob::State *qompl) override;
    virtual Config OMPLStateToConfig(const ob::State *qompl);
    virtual void print(std::ostream& out = std::cout) const;
    virtual double GetTime(const ob::State *qompl) override;
    virtual bool isTimeDependent() override;
  protected:
    uint N;
};

