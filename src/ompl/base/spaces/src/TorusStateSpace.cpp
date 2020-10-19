#include "ompl/base/spaces/TorusStateSpace.h"
#include "ompl/tools/config/MagicConstants.h"
#include <cstring>

using namespace ompl::base;

ompl::base::TorusStateSpace::TorusStateSpace()
{
    setName("Torus" + getName());
    type_ = STATE_SPACE_UNKNOWN;
    addSubspace(std::make_shared<SO2StateSpace>(), 1.0);
    addSubspace(std::make_shared<SO2StateSpace>(), 1.0);
    lock();
}

double TorusStateSpace::distance(const State *state1, const State *state2) const
{
    const auto *cstate1 = static_cast<const CompoundState *>(state1);
    const auto *cstate2 = static_cast<const CompoundState *>(state2);
    double x = components_[0]->distance(cstate1->components[0], cstate2->components[0]);
    double y = components_[1]->distance(cstate1->components[1], cstate2->components[1]);
    return sqrtf(x*x + y*y);
}

ompl::base::State *ompl::base::TorusStateSpace::allocState() const
{
    auto *state = new StateType();
    allocStateComponents(state);
    return state;
}

void ompl::base::TorusStateSpace::freeState(State *state) const
{
    CompoundStateSpace::freeState(state);
}

