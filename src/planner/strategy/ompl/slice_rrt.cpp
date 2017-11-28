#include "slice_rrt.h"

using namespace ompl::geometric;

SliceRRT::SliceRRT(const ob::SpaceInformationPtr &C_, const ob::SpaceInformationPtr &M_, SliceRRT *previous_): 
  PlainRRT(C), C(C_), M(M_), previous(previous_)
{
  setName("SliceRRT");
}
SliceRRT::~SliceRRT(void)
{
}

