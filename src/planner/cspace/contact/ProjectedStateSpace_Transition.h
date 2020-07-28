#pragma once
#include "ompl/base/MotionValidator.h"
#include "ompl/base/PlannerData.h"
#include "ompl/base/StateSampler.h"
#include "ompl/base/ValidStateSampler.h"
#include "ompl/base/Constraint.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"
#include "ompl/base/spaces/constraint/ConstrainedStateSpace.h"
#include <ompl/base/spaces/constraint/ProjectedStateSpace.h>
#include <ompl/util/RandomNumbers.h>
#include "planner/cspace/contact/TransitionModeTypes.h"

#include <Eigen/Core>


// COPY OF PROJECTEDSTATESPACE from OMPL library

namespace ompl
{
    namespace base
    {
        /// @cond IGNORE
        /** \brief Forward declaration of ompl::base::ProjectedStateSpace */
        OMPL_CLASS_FORWARD(ProjectedStateSpaceTransition);
        /// @endcond

        /** \brief StateSampler for use for a projection-based state space. */
        class ProjectedStateSamplerTransition : public ProjectedStateSampler
        {
        public:
            /** \brief Constructor. */
            ProjectedStateSamplerTransition(const ProjectedStateSpaceTransition *space, StateSamplerPtr sampler);

            /** \brief Sample a state uniformly in ambient space and project to
             * the manifold. Return sample in \a state. */
            void sampleUniform(State *state) override;
            void sampleUniformNear(State *state, const State *near, double distance) override;
            void sampleGaussian(State *state, const State *mean, double stdDev) override;


        protected:
            /** \brief Constraint. */
            const ConstraintPtr constraint_;
            std::vector<ConstraintPtr> constraintsVec;
            RNG randomNumberGenerator;
        };


        /** \brief ConstrainedStateSpace encapsulating a projection-based
         * methodology for planning with constraints. */
        class ProjectedStateSpaceTransition : public ConstrainedStateSpace
        {
        public:
            /** \brief Construct an atlas with the specified dimensions. */
            ProjectedStateSpaceTransition(const StateSpacePtr &ambientSpace, const ConstraintPtr &constraint)
              : ConstrainedStateSpace(ambientSpace, constraint)
            {
                setName("Projected" + space_->getName());
            }

            /** \brief Destructor. */
            ~ProjectedStateSpaceTransition() override = default;

            /** \brief Allocate the default state sampler for this space. */
            StateSamplerPtr allocDefaultStateSampler() const override
            {
                return std::make_shared<ProjectedStateSamplerTransition>(this, space_->allocDefaultStateSampler());
            }

            void setMode(TransitionMode mode);

            /** \brief Allocate the previously set state sampler for this space. */
            StateSamplerPtr allocStateSampler() const override
            {
                return std::make_shared<ProjectedStateSamplerTransition>(this, space_->allocStateSampler());
            }

            /** \brief Traverse the manifold from \a from toward \a to. Returns
             * true if we reached \a to, and false if we stopped early for any
             * reason, such as a collision or traveling too far. No collision
             * checking is performed if \a interpolate is true. If \a geodesic
             * is not nullptr, the sequence of intermediates is saved to it,
             * including a copy of \a from, as well as the final state, which is
             * a copy of \a to if we reached \a to. Caller is responsible for
             * freeing states returned in \a geodesic.*/
            bool discreteGeodesic(const State *from, const State *to, bool interpolate = false,
                                  std::vector<State *> *geodesic = nullptr) const override;
            // void resetTransitionConstraints();

            // unsigned int validSegmentCount(const State *state1, const State *state2) const override;

        };
    }
}
