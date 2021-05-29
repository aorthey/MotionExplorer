#pragma once
#include "ompl/base/MotionValidator.h"
#include "ompl/base/DiscreteMotionValidator.h"
#include "ompl/base/SpaceInformation.h"

class CSpaceOMPLMultiAgent;

namespace ompl
{
    namespace base
    {
        class TimeMotionValidator: public DiscreteMotionValidator
        {
            using BaseT = DiscreteMotionValidator;
        public:

            TimeMotionValidator(SpaceInformation *si, CSpaceOMPLMultiAgent*);
            TimeMotionValidator(const SpaceInformationPtr &si, CSpaceOMPLMultiAgent*);

            ~TimeMotionValidator() override = default;

            // bool checkSpeedConstraint(const State *s1, const State *s2) const;

            bool checkMotion(const State *s1, const State *s2) const override;

            bool checkMotion(const State *s1, const State *s2, std::pair<State *, double> &lastValid) const override;

        protected:
            CSpaceOMPLMultiAgent *cspace_;
        };
    }
}



