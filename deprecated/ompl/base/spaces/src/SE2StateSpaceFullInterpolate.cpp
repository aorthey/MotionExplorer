#include "ompl/base/spaces/SE2StateSpaceFullInterpolate.h"
#include "ompl/tools/config/MagicConstants.h"
//#include <cstring>

ompl::base::State *ompl::base::SE2StateSpaceFullInterpolate::allocState() const
{
    auto *state = new StateType();
    allocStateComponents(state);
    return state;
}

void ompl::base::SE2StateSpaceFullInterpolate::freeState(State *state) const
{
    CompoundStateSpace::freeState(state);
}

void ompl::base::SE2StateSpaceFullInterpolate::registerProjections()
{
    class SE2DefaultProjection : public ProjectionEvaluator
    {
    public:
        SE2DefaultProjection(const StateSpace *space) : ProjectionEvaluator(space)
        {
        }

        unsigned int getDimension() const override
        {
            return 2;
        }

        void defaultCellSizes() override
        {
            cellSizes_.resize(2);
            bounds_ = space_->as<SE2StateSpaceFullInterpolate>()->getBounds();
            cellSizes_[0] = (bounds_.high[0] - bounds_.low[0]) / magic::PROJECTION_DIMENSION_SPLITS;
            cellSizes_[1] = (bounds_.high[1] - bounds_.low[1]) / magic::PROJECTION_DIMENSION_SPLITS;
        }

        void project(const State *state, Eigen::Ref<Eigen::VectorXd> projection) const override
        {
            projection = Eigen::Map<const Eigen::VectorXd>(
                state->as<SE2StateSpaceFullInterpolate::StateType>()->as<RealVectorStateSpace::StateType>(0)->values, 2);
        }
    };

    registerDefaultProjection(std::make_shared<SE2DefaultProjection>(this));
}

