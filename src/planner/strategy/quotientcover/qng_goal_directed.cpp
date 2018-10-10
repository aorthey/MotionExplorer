#include "common.h"
#include "qng_goal_directed.h"

using namespace ompl::geometric;

QNGGoalDirected::QNGGoalDirected(const base::SpaceInformationPtr &si, Quotient *parent ): BaseT(si, parent)
{
  setName("QNGGoalDirected"+std::to_string(id));
}

QNGGoalDirected::~QNGGoalDirected(void)
{
}

