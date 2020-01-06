#include "planner/cspace/cspace_geometric.h"

class GeometricCSpaceOMPLRN: public GeometricCSpaceOMPL
{
  public:
    GeometricCSpaceOMPLRN(RobotWorld *world_, int robot_id, int dimension);
    virtual void initSpace();
    virtual void ConfigToOMPLState(const Config &q, ob::State *qompl) override;
    virtual Config OMPLStateToConfig(const ob::State *qompl);
    virtual void print(std::ostream& out = std::cout) const;
    virtual Vector3 getXYZ(const ob::State*) override;
  protected:
    uint N;
};

