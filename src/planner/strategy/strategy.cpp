#include "planner/strategy/strategy.h"

#include <ompl/base/ValidStateSampler.h>
#include <ompl/base/samplers/UniformValidStateSampler.h>
#include <ompl/base/samplers/GaussianValidStateSampler.h>
#include <ompl/base/samplers/ObstacleBasedValidStateSampler.h>
#include <ompl/base/samplers/MinimumClearanceValidStateSampler.h>
#include <ompl/base/samplers/MaximizeClearanceValidStateSampler.h>
#include <ompl/base/samplers/BridgeTestValidStateSampler.h>

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

void Strategy::BenchmarkFileToPNG(const std::string &file)
{
  std::string cmd;

  cmd = "ompl_benchmark_statistics.py "+file+".log -d "+file+".db";
  int s1 = std::system(cmd.c_str());

  cmd = "cp "+file+".db"+" ../data/benchmarks/";
  int s2 = std::system(cmd.c_str());

  cmd = "python ../scripts/ompl_output_benchmark.py "+file+".db";
  int s3 = std::system(cmd.c_str());

  cmd = "python ../scripts/ompl_benchmark_statistics_simple.py "+file+".log -d "+file+".db -p "+file+".pdf";
  int s4 = std::system(cmd.c_str());

  cmd = "convert -density 150 "+file+".pdf -trim -quality 100 "+file+".png";
  int s5 = std::system(cmd.c_str());

  cmd = "eog "+file+".png";
  int s6 = std::system(cmd.c_str());

  if(s1&s2&s3&s4&s5&s6){
    std::cout << "Successfully wrote benchmark to " << file << ".png" << std::endl;
  }else{
    std::cout << "benchmark to png failed" << std::endl;
  }
}
