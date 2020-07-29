#pragma once
#include "planner/cspace/contact/ConstraintIntersectionMultiMode.h"
#include "ompl/base/MotionValidator.h"
#include "ompl/base/PlannerData.h"
#include "ompl/base/StateSampler.h"
#include "ompl/base/ValidStateSampler.h"
#include "ompl/base/Constraint.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"
#include "ompl/base/spaces/constraint/ConstrainedStateSpace.h"
#include <ompl/base/spaces/constraint/ProjectedStateSpace.h>
#include <ompl/util/RandomNumbers.h>

#include <Eigen/Core>


// COPY OF PROJECTEDSTATESPACE from OMPL library

namespace ompl
{
    namespace base
    {
        /// @cond IGNORE
        /** \brief Forward declaration of ompl::base::ProjectedStateSpace */
        OMPL_CLASS_FORWARD(ProjectedStateSpaceMultiMode);
        /// @endcond

        /** \brief StateSampler for use for a projection-based state space. */
        class ProjectedStateSamplerMultiMode : public ProjectedStateSampler
        {
        public:
            /** \brief Constructor. */
            ProjectedStateSamplerMultiMode(const ProjectedStateSpaceMultiMode *space, StateSamplerPtr sampler);

            /** \brief Sample a state uniformly in ambient space and project to
             * the manifold. Return sample in \a state. */
            void sampleUniform(State *state) override;
            void sampleUniformNear(State *state, const State *near, double distance) override;
            void sampleGaussian(State *state, const State *mean, double stdDev) override;


        protected:
            /** \brief Constraint. */
            const ConstraintPtr constraint_;
            std::vector<ConstraintPtr> constraintsVec;
            ConstraintIntersectionMultiModePtr constraintIntersection_;
        };

        /** \brief ConstrainedStateSpace encapsulating a projection-based
         * methodology for planning with constraints. */
        class ProjectedStateSpaceMultiMode : public ConstrainedStateSpace
        {
        public:
            /** \brief Construct an atlas with the specified dimensions. */
            ProjectedStateSpaceMultiMode(const StateSpacePtr &ambientSpace, const ConstraintPtr &constraint)
              : ConstrainedStateSpace(ambientSpace, constraint)
            {
                setName("Projected" + space_->getName());
                // constraintIntersection_ =
                //   std::dynamic_pointer_cast<const ConstraintIntersectionMultiMode>(constraint);
                // if(constraintIntersection_ == nullptr)
                // {
                //   throw ompl::Exception("Required Intersection Constraint MultiMode");
                // }
            }

            /** \brief Destructor. */
            ~ProjectedStateSpaceMultiMode() override = default;

            class StateType : public ConstrainedStateSpace::StateType
            {
            public:
                /** \brief Construct state of size \a n. */
                StateType(const ConstrainedStateSpace *space) : ConstrainedStateSpace::StateType(space)
                {
                }
                void setMode(ConstraintMode mode)
                {
                  mode_ = mode;
                }
                ConstraintMode getMode() const
                {
                  return mode_;
                }
            protected:
                ConstraintMode mode_; //to which constraint intersection the state belongs

            };


            /** \brief Allocate the default state sampler for this space. */
            StateSamplerPtr allocDefaultStateSampler() const override
            {
                return std::make_shared<ProjectedStateSamplerMultiMode>(this, space_->allocDefaultStateSampler());
            }

            /** \brief Allocate the previously set state sampler for this space. */
            StateSamplerPtr allocStateSampler() const override
            {
                return std::make_shared<ProjectedStateSamplerMultiMode>(this, space_->allocStateSampler());
            }
            void copyState(State *destination, const State *source) const override
            {
                ConstrainedStateSpace::copyState(destination, source);
                destination->as<StateType>()->setMode(source->as<StateType>()->getMode());
            }
            State *allocState() const override
            {
                return new StateType(this);
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

            // unsigned int validSegmentCount(const State *state1, const State *state2) const override;


        };
    }
}
