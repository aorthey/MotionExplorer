#include "planner/cspace/cspace_geometric.h"

class GeometricCSpaceOMPLSE2RN: public GeometricCSpaceOMPL
{
  public:
    GeometricCSpaceOMPLSE2RN(RobotWorld *world_, int robot_idx);
    virtual void initSpace();
    virtual void ConfigToOMPLState(const Config &q, ob::State *qompl) override;
    virtual Config OMPLStateToConfig(const ob::State *qompl) override;
    virtual void print(std::ostream& out = std::cout) const;
};

