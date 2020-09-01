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

#include "ompl/base/DynamicalMotionValidator.h"
#include "ompl/control/SpaceInformation.h"
#include "ompl/util/Exception.h"
#include <queue>

void ompl::base::DynamicalMotionValidator::defaultSettings()
{
    isDynamic = false;
    siC = dynamic_cast<ompl::control::SpaceInformation*>(si_);
    if(siC != nullptr) {
      isDynamic = true;
      controlDuration_= 0 ;
      c_current = siC->allocControl();
      sampler = siC->allocSimpleDirectedControlSampler();
      sampler->setNumControlSamples(50);
	  //std::cout << sampler->getNumControlSamples() << std::endl;      
      s_target_copy = siC->allocState();
    } 
}

bool ompl::base::DynamicalMotionValidator::checkMotion(const State *s1, const State *s2) const
{
    /* assume motion starts in a valid configuration so s1 is valid */
    
    if (!si_->isValid(s2))
    {
        invalid_++;
        return false;
    }

    if(!isDynamic){
        return BaseT::checkMotion(s1, s2);
    } else {
        si_->copyState(s_target_copy, s2);
        unsigned int c_t =0 ;
        auto* p_this = const_cast<DynamicalMotionValidator*>(this);
        p_this->controlDuration_=sampler->sampleTo(c_current, s1, s_target_copy);
        double targetRegion = tolerance * si_->getStateSpace()->distance(s1, s2);
        return (si_->getStateSpace()->distance(s_target_copy, s2) <= targetRegion);
    }
}

bool ompl::base::DynamicalMotionValidator::checkMotion(const State *s1, const State *s2,
                                                       std::pair<State *, double> &lastValid) const
{
  if(!isDynamic){
    return BaseT::checkMotion(s1, s2, lastValid);
  } else{
    OMPL_ERROR("NYI");
    throw "NYI";
  }
}



