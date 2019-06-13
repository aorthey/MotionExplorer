#pragma once
#include "ompl/base/spaces/SO2StateSpace.h"
#include <boost/math/constants/constants.hpp>


namespace ompl
{
    namespace base
    {
        class SO2StateSpaceFullInterpolate : public SO2StateSpace
        {

          public:
            //Required to extrapolate into a certain direction (used in
            //quotientchart to extrapolate from a center to the boundary of a
            //neighborhood set)
            void interpolate(const State *from, const State *to, const double t, State *state) const override
            {
                const double &pi = boost::math::constants::pi<double>();
                double &v = state->as<StateType>()->value;
                double diff = to->as<StateType>()->value - from->as<StateType>()->value;

                if (fabs(diff) <= pi)
                    v = from->as<StateType>()->value + diff * t;
                else
                {
                    if (diff > 0.0) diff = +2.0*pi - diff;
                    else diff = -2.0*pi - diff;

                    v = from->as<StateType>()->value - diff * t;
                }
                while(v > +pi) v-= 2*pi;
                while(v < -pi) v+= 2*pi;
            }
        };
    }
}

