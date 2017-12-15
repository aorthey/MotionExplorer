#pragma once

#include <ompl/base/ValidStateSampler.h>
#include <ompl/base/StateSampler.h>

namespace ompl
{
  namespace base
  {
    /** \brief Generate valid samples using bridge test. First
        sample an invalid state, then sample another invalid state. Take the midpoint of those samples. 
        If midpoint is valid, return. If midpoint is invalid continue.

        @par External documentation
        Hsu, D., Jiang, T., Reif, J., & Sun, Z., The bridge test for sampling narrow
        passages with probabilistic roadmap planners. In <em>Robotics and
        Automation</em>, 2003. 
        [[URL]](ftp://www-cs-faculty.stanford.edu/cs/robotics/pub/dyhsu/papers/icra03.pdf)

    */

    class BridgeTestValidStateSampler : public ValidStateSampler
    {
    public:
      /** \brief Constructor */
      BridgeTestValidStateSampler(const SpaceInformation *si);

      ~BridgeTestValidStateSampler() override = default;

      bool sample(State *state) override;
      bool sampleNear(State *state, const State *near, const double distance) override;

      /** \brief Get the standard deviation used when sampling */
      double getStdDev() const
      {
          return stddev_;
      }

      /** \brief Set the standard deviation to use when sampling */
      void setStdDev(double stddev)
      {
          stddev_ = stddev;
      }


    protected:
      /** \brief The sampler to build upon */
      StateSamplerPtr sampler_;

      /** \brief The standard deviation to use in the sampling process */
      double stddev_;



    };
  }
}
