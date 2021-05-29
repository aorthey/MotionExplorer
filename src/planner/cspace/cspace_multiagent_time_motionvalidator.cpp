#include "cspace_multiagent_time_motionvalidator.h"
#include "cspace_multiagent.h"
#include <numeric>

using namespace ompl::base;

TimeMotionValidator::TimeMotionValidator(SpaceInformation *si, CSpaceOMPLMultiAgent* cspace):
  BaseT(si), cspace_(cspace)
{
}

TimeMotionValidator::TimeMotionValidator(const SpaceInformationPtr &si, CSpaceOMPLMultiAgent* cspace):
  BaseT(si), cspace_(cspace)
{
}

// bool AnimationMotionValidator::checkSpeedConstraint(const State *s1, const State *s2) const
// {
//     const double t1 = s1->as<base::AnimationSpace::StateType>()->getTime();
//     const double t2 = s2->as<base::AnimationSpace::StateType>()->getTime();

//     if(t1 > t2) return false;

//     auto const aspace = std::static_pointer_cast<const AnimationSpace>(si_->getStateSpace());
//     double dt = aspace->distanceTime(s1, s2);
//     double dtmin = aspace->getMinimalTimeToReach(s1, s2);
//     return (dtmin <= dt);
// }

bool TimeMotionValidator::checkMotion(const State *s1, const State *s2,
                                                      std::pair<State *, double> &lastValid) const
{
    double t1 = cspace_->GetTime(s1);
    double t2 = cspace_->GetTime(s2);
    if(t1 <= t2) return BaseT::checkMotion(s1, s2, lastValid);
    else return false;
}

bool TimeMotionValidator::checkMotion(const State *s1, const State *s2) const
{
    double t1 = cspace_->GetTime(s1);
    double t2 = cspace_->GetTime(s2);
    if(t1 <= t2) return BaseT::checkMotion(s1, s2);
    else return false;
}
