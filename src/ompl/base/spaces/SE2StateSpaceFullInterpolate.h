#pragma once
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include "ompl/base/spaces/SO2StateSpaceFullInterpolate.h"

namespace ompl
{
    namespace base
    {
        class SE2StateSpaceFullInterpolate : public CompoundStateSpace
        {
        public:
          typedef SE2StateSpace::StateType StateType;

            SE2StateSpaceFullInterpolate()
            {
              setName("SE2" + getName());
              type_ = STATE_SPACE_SE2;
              addSubspace(std::make_shared<RealVectorStateSpace>(2), 1.0);
              addSubspace(std::make_shared<SO2StateSpaceFullInterpolate>(), 0.5);
              lock();
            }

            ~SE2StateSpaceFullInterpolate() override = default;

            /** \copydoc RealVectorStateSpace::setBounds() */
            void setBounds(const RealVectorBounds &bounds)
            {
                as<RealVectorStateSpace>(0)->setBounds(bounds);
            }

            /** \copydoc RealVectorStateSpace::getBounds() */
            const RealVectorBounds &getBounds() const
            {
                return as<RealVectorStateSpace>(0)->getBounds();
            }

            State *allocState() const override;
            void freeState(State *state) const override;

            void registerProjections() override;
        };
    }
}
