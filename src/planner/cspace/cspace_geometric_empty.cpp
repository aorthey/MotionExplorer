#include "planner/cspace/cspace_geometric_empty.h"
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/DiscreteStateSpace.h>

namespace ompl
{
    namespace base
    {
        class EmptyStateSpace : public RealVectorStateSpace
        {
        public:
            EmptyStateSpace()
              : RealVectorStateSpace(0)
            {
                setName("EmptySpace");
            }
            ~EmptyStateSpace() override = default;
            double getMeasure() const{
              return 1;
            }
            void setup() override{
              return;
            }
        };
    }
}


GeometricCSpaceOMPLEmpty::GeometricCSpaceOMPLEmpty(RobotWorld *world_):
  GeometricCSpaceOMPL(world_, -1)
{
}

void GeometricCSpaceOMPLEmpty::initSpace()
{
  ob::StateSpacePtr RN = std::make_shared<ob::EmptyStateSpace>();
  // ob::StateSpacePtr RN = (std::make_shared<ob::DiscreteStateSpace>(0,0));
  this->space = RN;
  // ob::RealVectorStateSpace *cspace = this->space->as<ob::RealVectorStateSpace>();
}
uint GeometricCSpaceOMPLEmpty::GetDimensionality() const{
  return 0;
}
uint GeometricCSpaceOMPLEmpty::GetKlamptDimensionality() const{
  return 0;
}

void GeometricCSpaceOMPLEmpty::ConfigToOMPLState(const Config &q, ob::State *qompl)
{
}

Config GeometricCSpaceOMPLEmpty::OMPLStateToConfig(const ob::State *qompl){
  Config q;
  return q;
}
void GeometricCSpaceOMPLEmpty::print(std::ostream& out) const
{
  out << std::string(80, '-') << std::endl;
  out << "Empty ConfigurationSpace" << std::endl;
  out << std::string(80, '-') << std::endl;
}


Vector3 GeometricCSpaceOMPLEmpty::getXYZ(const ob::State *s)
{
  Vector3 q(0,0,0);
  return q;
}
