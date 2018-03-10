#include "planner/strategy/strategy.h"

#include "ompl/BridgeTestValidStateSampler.h"

#include <ompl/base/ValidStateSampler.h>
#include <ompl/base/samplers/UniformValidStateSampler.h>
#include <ompl/base/samplers/GaussianValidStateSampler.h>
#include <ompl/base/samplers/ObstacleBasedValidStateSampler.h>
#include <ompl/base/samplers/MinimumClearanceValidStateSampler.h>
#include <ompl/base/samplers/MaximizeClearanceValidStateSampler.h>

ob::ValidStateSamplerPtr allocUniformValidStateSampler(const ob::SpaceInformation *si)
{
  return std::make_shared<ob::UniformValidStateSampler>(si);
}
ob::ValidStateSamplerPtr allocGaussianValidStateSampler(const ob::SpaceInformation *si)
{
  return std::make_shared<ob::GaussianValidStateSampler>(si);
}
ob::ValidStateSamplerPtr allocMinimumClearanceValidStateSampler(const ob::SpaceInformation *si)
{
  return std::make_shared<ob::MinimumClearanceValidStateSampler>(si);
}
ob::ValidStateSamplerPtr allocMaximizeClearanceValidStateSampler(const ob::SpaceInformation *si)
{
  return std::make_shared<ob::MaximizeClearanceValidStateSampler>(si);
}
ob::ValidStateSamplerPtr allocObstacleBasedValidStateSampler(const ob::SpaceInformation *si)
{
  return std::make_shared<ob::ObstacleBasedValidStateSampler>(si);
}
ob::ValidStateSamplerPtr allocBridgeTestValidStateSampler(const ob::SpaceInformation *si)
{
  return std::make_shared<ob::BridgeTestValidStateSampler>(si);
}

Strategy::Strategy()
{
}

void Strategy::setStateSampler(std::string sampler, ob::SpaceInformationPtr si)
{
  ob::ValidStateSamplerAllocator allocator;
  if(sampler=="uniform"){
    allocator = allocUniformValidStateSampler;
  }else if(sampler=="gaussian"){
    allocator = allocGaussianValidStateSampler;
  }else if(sampler=="minimum_clearance"){
    allocator = allocMinimumClearanceValidStateSampler;
  }else if(sampler=="maximum_clearance"){
    allocator = allocMinimumClearanceValidStateSampler;
  }else if(sampler=="obstacle_based"){
    allocator = allocObstacleBasedValidStateSampler;
  }else if(sampler=="bridge" || sampler=="bridge_test"){
    allocator = allocBridgeTestValidStateSampler;
  }else{
    std::cout << "Sampler  " << sampler << " is unknown." << std::endl;
    exit(0);
  }
  si->clearValidStateSamplerAllocator();
  si->setValidStateSamplerAllocator(allocator);
}

