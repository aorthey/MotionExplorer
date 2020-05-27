/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, University of Stuttgart
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
 *   * Neither the name of the University of Stuttgart nor the names
 *     of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written
 *     permission.
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

/* Author: Andreas Orthey */

#include "util.h"

#include "hypercube/MultiLevelPlanningCommon.h"
#include "hypercube/MultiLevelPlanningHyperCubeCommon.h"
#include <ompl/util/Console.h>


double runHyperCubeBenchmark(int ndim, double maxTime, int Nruns)
{
  double averageTime = 0;
  for(int k = 0; k < Nruns; k++)
  {
    auto space(std::make_shared<ompl::base::RealVectorStateSpace>(ndim));
    ompl::base::RealVectorBounds bounds(ndim);
    ompl::geometric::SimpleSetup ss(space);
    ob::SpaceInformationPtr si = ss.getSpaceInformation();
    ob::ProblemDefinitionPtr pdef = ss.getProblemDefinition();
    ompl::base::ScopedState<> start(space), goal(space);

    bounds.setLow(0.);
    bounds.setHigh(1.);
    space->setBounds(bounds);
    ss.setStateValidityChecker(std::make_shared<HyperCubeValidityChecker>(si, ndim));
    for (int i = 0; i < ndim; ++i)
    {
        start[i] = 0.;
        goal[i] = 1.;
    }
    ss.setStartAndGoalStates(start, goal);

    std::vector<int> admissibleProjection = getHypercubeAdmissibleProjection(ndim);

    // ob::PlannerPtr planner = GetMultiLevelPlanner<og::QRRTStar>(admissibleProjection, si, "QRRTStar");
    ob::PlannerPtr planner = std::make_shared<og::STRIDE>(si);
    ss.setPlanner(planner);

    bool solved = ss.solve(maxTime);

    if(!solved)
    {
      std::cout << "Not solved:" << k << "/" << Nruns << " dim:" << ndim << " maxTime:" << maxTime << std::endl;
    }

    double timeToCompute = ss.getLastPlanComputationTime();

    averageTime += timeToCompute;
  }
  averageTime = averageTime / Nruns;
  return averageTime;
}

int main(int argc, char **argv)
{
    const unsigned int startDim = 3;
    const unsigned int endDim = 10;
    const unsigned int Nruns = 10;
    const double maxTime = 300.0;

    ompl::msg::setLogLevel(ompl::msg::LogLevel::LOG_NONE);

    std::cout << "x = np.array([";
    for(unsigned int k = startDim; k < endDim; k++)
    {
      std::cout << k;
      if(k<endDim-1) std::cout << ", ";
    }
    std::cout << "])" << std::endl;

    std::cout << "time = np.array([" << std::flush;
    for(unsigned int k = startDim; k < endDim; k++)
    {
       double timek = runHyperCubeBenchmark(k, maxTime, Nruns);
       std::cout << timek << std::flush;

       if(k<endDim-1) std::cout << ", " << std::flush;
    }
    std::cout << "])" << std::endl;

    return 0;
}

