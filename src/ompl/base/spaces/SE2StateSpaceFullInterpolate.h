#pragma once
#include <ompl/base/spaces/SE2StateSpace.h>
#include "ompl/base/spaces/SO2StateSpaceFullInterpolate.h"

namespace ompl
{
    namespace base
    {
        class SE2StateSpaceFullInterpolate : public SE2StateSpace
        {

          public:
            SE2StateSpaceFullInterpolate()
            {
              setName("SE2" + getName());
              type_ = STATE_SPACE_SE2;
              addSubspace(std::make_shared<RealVectorStateSpace>(2), 1.0);
              addSubspace(std::make_shared<SO2StateSpaceFullInterpolate>(), 0.5);
              lock();
            }
        };
    }
}

