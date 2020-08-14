/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, Rice University
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Rice University nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Ioan Sucan */

#ifndef OMPL_BASE_DYNAMICAL_MOTION_VALIDATOR_
#define OMPL_BASE_DYNAMICAL_MOTION_VALIDATOR_

#include "ompl/base/DiscreteMotionValidator.h"
#include "ompl/base/SpaceInformation.h"
#include "ompl/control/Control.h"
#include "ompl/control/SpaceInformation.h"
#include "ompl/control/DirectedControlSampler.h"
#include "ompl/control/SimpleDirectedControlSampler.h"

namespace ompl
{
    namespace base
    {
        /** \brief A motion validator that only uses the state validity checker. Motions are checked for validity at a
         * specified resolution. */
        class DynamicalMotionValidator : public DiscreteMotionValidator
        {
            using BaseT = ompl::base::DiscreteMotionValidator;

        public:
            /** \brief Constructor */
            DynamicalMotionValidator(SpaceInformation *si) : DiscreteMotionValidator(si)
            {
                defaultSettings();
            }

            /** \brief Constructor */
            DynamicalMotionValidator(const SpaceInformationPtr &si) : DiscreteMotionValidator(si)
            {
                defaultSettings();
            }

            ~DynamicalMotionValidator() override = default;

            bool checkMotion(const State *s1, const State *s2) const override ;

            bool checkMotion(const State *s1, const State *s2, std::pair<State *, double> &lastValid) const override;
            
            ompl::control::Control* getCurrentControl() const
            {
              return c_current ;
            }
			
            double getControlDuration() const
            {
              return controlDuration_ ;
            }

        protected:

            void defaultSettings();
	    
            ompl::control::SimpleDirectedControlSamplerPtr sampler;
            ompl::control::Control* c_current;
            double controlDuration_ ;
            ompl::base::State* s_target_copy;
            ompl::control::SpaceInformation* siC;
            bool isDynamic; 
            double tolerance{0.05}; //multiplies with distance of start and target
        };
    }
}

#endif
