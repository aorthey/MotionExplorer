#include "../SimpleDirectedControlSampler.h"
#include "ompl/control/SpaceInformation.h"

ompl::control::SimpleDirectedControlSampler::SimpleDirectedControlSampler(const SpaceInformation *si, unsigned int k)
  : DirectedControlSampler(si), cs_(si->allocControlSampler()), numControlSamples_(k)
{
}

ompl::control::SimpleDirectedControlSampler::~SimpleDirectedControlSampler() = default;

unsigned int ompl::control::SimpleDirectedControlSampler::sampleTo(Control *control, const base::State *source,
                                                                   base::State *dest)
{
    return getBestControl(control, source, dest, nullptr);
}

unsigned int ompl::control::SimpleDirectedControlSampler::sampleTo(Control *control, const Control *previous,
                                                                   const base::State *source, base::State *dest)
{
    return getBestControl(control, source, dest, previous);
}

unsigned int ompl::control::SimpleDirectedControlSampler::getBestControl(Control *control, const base::State *source,
                                                                         base::State *dest, const Control *previous)
{
    // Sample the first control
    if (previous != nullptr)
        cs_->sampleNext(control, previous, source);
    else
        cs_->sample(control, source);

    const unsigned int minDuration = si_->getMinControlDuration();
    const unsigned int maxDuration = si_->getMaxControlDuration();
    // std::cout << minDuration << "," << maxDuration << std::endl;

    unsigned int steps = cs_->sampleStepCount(minDuration, maxDuration);
    // Propagate the first control, and find how far it is from the target state
    base::State *bestState = si_->allocState();
    steps = si_->propagateWhileValid(source, control, steps, bestState);

//###############################################
    dist = si_->distance(source,dest);
    //if((si_->getPropagationStepSize() * maxDuration) < (0.75*dist)){
    //    std::cout << "too far away" << std::endl;
    //    si_->copyState(dest, bestState);
    //    si_->freeState(bestState);
    //    return steps;
    //}
//###############################################

    if (numControlSamples_ > 1)
    {
        Control *tempControl = si_->allocControl();
        base::State *tempState = si_->allocState();
        double bestDistance = si_->distance(bestState, dest);
      //########################################
        toleratedDistance_ = distanceFactor_ * dist;
      //#######################################
        // Sample k-1 more controls, and save the control that gets closest to target
        for (unsigned int i = 1; i < numControlSamples_; ++i)
        {
            unsigned int sampleSteps = cs_->sampleStepCount(minDuration, maxDuration);
            if (previous != nullptr)
                cs_->sampleNext(tempControl, previous, source);
            else
                cs_->sample(tempControl, source);

            sampleSteps = si_->propagateWhileValid(source, tempControl, sampleSteps, tempState);
            double tempDistance = si_->distance(tempState, dest);
            if (tempDistance < bestDistance)
            {
                si_->copyState(bestState, tempState);
                si_->copyControl(control, tempControl);
                bestDistance = tempDistance;
                steps = sampleSteps;
            //###########################################		
                if(bestDistance < toleratedDistance_){
                  si_->freeState(tempState);
                  si_->freeControl(tempControl);
                  // std::cout << "Exited with " << i << " samples and " << steps << " steps" << std::endl;
                  return steps;
                }
            //##########################################
            }
        }

        si_->freeState(tempState);
        si_->freeControl(tempControl);
    }

    // std::cout << "checked Motion not feasible" << std::endl;

    si_->copyState(dest, bestState);
    si_->freeState(bestState);

    return steps;
}
