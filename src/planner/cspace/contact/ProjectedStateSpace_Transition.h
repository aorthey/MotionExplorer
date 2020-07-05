#pragma once
#include "ompl/base/MotionValidator.h"
#include "ompl/base/PlannerData.h"
#include "ompl/base/StateSampler.h"
#include "ompl/base/ValidStateSampler.h"
#include "ompl/base/Constraint.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"
#include "ompl/base/spaces/constraint/ConstrainedStateSpace.h"

#include <Eigen/Core>

namespace ompl
{
    namespace base
    {
        /// @cond IGNORE
        /** \brief Forward declaration of ompl::base::ProjectedStateSpace */
        OMPL_CLASS_FORWARD(ProjectedStateSpace);
        /// @endcond

        /** \brief StateSampler for use for a projection-based state space. */
        class ProjectedStateSampler : public WrapperStateSampler
        {
        public:
            /** \brief Constructor. */
            ProjectedStateSampler(const ProjectedStateSpace *space, StateSamplerPtr sampler);

            /** \brief Sample a state uniformly in ambient space and project to
             * the manifold. Return sample in \a state. */
            void sampleUniform(State *state) override;

            /** \brief Sample a state uniformly from the ball with center \a
             * near and radius \a distance in ambient space and project to the
             * manifold. Return sample in \a state. */
            void sampleUniformNear(State *state, const State *near, double distance) override;

            /** \brief Sample a state uniformly from a normal distribution with
                given \a mean and \a stdDev in ambient space and project to the
                manifold. Return sample in \a state. */
            void sampleGaussian(State *state, const State *mean, double stdDev) override;

        protected:
            /** \brief Constraint. */
            const ConstraintPtr constraint_;
        };

        /**
           @anchor gProject
           @par Short Description
           ProjectedStateSpace implements a projection-based methodology for constrained sampling-based planning, where
           points in ambient space are \e projected onto the constraint manifold via a projection operator, which in
           this case is implemented as a Newton's method within the Constraint.

           @par External Documentation
           For more information on constrained sampling-based planning using projection-based methods, see the following review paper.
           The section on projection-based methods cites most of the relevant literature.

           Z. Kingston, M. Moll, and L. E. Kavraki, “Sampling-Based Methods for Motion Planning with Constraints,”
           Annual Review of Control, Robotics, and Autonomous Systems, 2018. DOI:
           <a href="http://dx.doi.org/10.1146/annurev-control-060117-105226">10.1146/annurev-control-060117-105226</a>
           <a href="http://kavrakilab.org/publications/kingston2018sampling-based-methods-for-motion-planning.pdf">[PDF]</a>.
        */

        /** \brief ConstrainedStateSpace encapsulating a projection-based
         * methodology for planning with constraints. */
        class ProjectedStateSpace : public ConstrainedStateSpace
        {
        public:
            /** \brief Construct an atlas with the specified dimensions. */
            ProjectedStateSpace(const StateSpacePtr &ambientSpace, const ConstraintPtr &constraint)
              : ConstrainedStateSpace(ambientSpace, constraint)
            {
                setName("Projected" + space_->getName());
            }

            /** \brief Destructor. */
            ~ProjectedStateSpace() override = default;

            /** \brief Allocate the default state sampler for this space. */
            StateSamplerPtr allocDefaultStateSampler() const override
            {
                return std::make_shared<ProjectedStateSampler>(this, space_->allocDefaultStateSampler());
            }

            /** \brief Allocate the previously set state sampler for this space. */
            StateSamplerPtr allocStateSampler() const override
            {
                return std::make_shared<ProjectedStateSampler>(this, space_->allocStateSampler());
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
        };
    }
}

#endif
