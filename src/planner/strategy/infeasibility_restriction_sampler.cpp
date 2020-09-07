#include "infeasibility_restriction_sampler.h"
#include "util.h"


using namespace ompl::multilevel;
namespace ob = ompl::base;

RestrictionSamplerImpl::RestrictionSamplerImpl(
    const ompl::base::SpaceInformationPtr &si, BundleSpace *parent_):
  BaseT(si, parent_)
{
}

void RestrictionSamplerImpl::grow()
{
  if(hasTotalSpace()) return BaseT::grow();

  base::SpaceInformationPtr bundle = getBundle();
  BundleSpaceGraph *graph = static_cast<BundleSpaceGraph*>(getBaseBundleSpace());
  geometric::PathGeometric &spath = static_cast<geometric::PathGeometric &>(*graph->solutionPath_);
  std::vector<base::State *> states = spath.getStates();

  if (states.size() < 2)
  {
      OMPL_ERROR("No base path");
      return;
  }
  else
  {
      double endLength = spath.length();
      double distStopping = rng_.uniform01() * endLength;

      base::State *s1 = nullptr;
      base::State *s2 = nullptr;

      int ctr = 0;
      double distLastSegment = 0;
      double distCountedSegments = 0;
      while (distCountedSegments < distStopping && (ctr < (int)states.size() - 1))
      {
          s1 = states.at(ctr);
          s2 = states.at(ctr + 1);
          distLastSegment = getBase()->distance(s1, s2);
          distCountedSegments += distLastSegment;
          ctr++;
      }

      //          |---- d -----|
      //---O------O------------O
      //|--------- t ----------|
      //|--------- s ------|
      //          |d-(t-s) |
      double step = (distLastSegment - (distCountedSegments - distStopping)) / (distLastSegment);
      getBase()->getStateSpace()->interpolate(s1, s2, step, xBaseTmp_);
      sampleFiber(xFiberTmp_);
      liftState(xBaseTmp_, xFiberTmp_, xRandom_->state);

      bool found = getBundle()->getStateValidityChecker()->isValid(xRandom_->state);
      if(!found)
      {
        Configuration *q = new Configuration(getBundle(), xRandom_->state);
        addConfiguration(q);
      }
  }
}

void RestrictionSamplerImpl::clear()
{
  BaseT::clear();
}

void RestrictionSamplerImpl::setup()
{
  BaseT::setup();
}

void RestrictionSamplerImpl::getPlannerData(ob::PlannerData &data) const
{
  if(hasTotalSpace()) return;
  BaseT::getPlannerData(data);
}

