#include "qmp_sufficient.h"

#include <ompl/datastructures/PDF.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <boost/foreach.hpp>
#include <boost/graph/graphviz.hpp>

#include <boost/random/linear_congruential.hpp>
#include <boost/random/variate_generator.hpp>

using namespace og;
using namespace ob;
#define foreach BOOST_FOREACH

QMPSufficient::QMPSufficient(const ob::SpaceInformationPtr &si, QuotientChart *parent_ ):
  BaseT(si, parent_)
{
  setName("QMPSufficient"+std::to_string(id));
  number_of_samples = 0;
  number_of_infeasible_samples = 0;
  number_of_sufficient_samples = 0;

  checker = dynamic_pointer_cast<OMPLValidityCheckerNecessarySufficient>(si->getStateValidityChecker());
  if(!checker){
    checkSufficiency = false;
    //OMPL_WARN("Sufficient checker not set!");
  }else{
    checkSufficiency = true;
  }

}

QMPSufficient::~QMPSufficient()
{
}

void QMPSufficient::Grow(double t){
  ob::State *workState = xstates[0];
  iterations_++;
  bool found_feasible = Sample(workState);

  number_of_samples++;
  if(found_feasible)
  {
    if(checkSufficiency){
      bool sufficient = checker->IsSufficientFeasible(workState);
      if(sufficient){
        number_of_sufficient_samples++;
        double d = checker->SufficientDistance(workState);
        std::cout << d << std::endl;
      }
    }
  }else{
    number_of_infeasible_samples++;
  }
  std::cout << "Samples: " << number_of_samples << " (" 
    << 100.0*(double)number_of_infeasible_samples/(double)number_of_samples << "\% infeasible | "
    << 100.0*(double)(number_of_samples-number_of_infeasible_samples)/(double)number_of_samples << "\% feasible | "
    << 100.0*(double)number_of_sufficient_samples/(double)number_of_samples << "\% sufficient)"
    << std::endl;

}

bool QMPSufficient::Sample(ob::State *q_random)
{
  totalNumberOfSamples++;
  if(parent == nullptr){
    M1_sampler->sampleUniform(q_random);
  }else{
    //Adjusted sampling function: Sampling in G0 x C1
    ob::SpaceInformationPtr M0 = parent->getSpaceInformation();
    base::State *s_C1 = C1->allocState();
    base::State *s_M0 = M0->allocState();

    C1_sampler->sampleUniform(s_C1);
    parent->SampleGraph(s_M0);
    mergeStates(s_M0, s_C1, q_random);

    C1->freeState(s_C1);
    M0->freeState(s_M0);
  }
  return M1->isValid(q_random);
}
