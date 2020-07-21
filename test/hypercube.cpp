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
#include <ompl/base/spaces/RealVectorStateSpace.h>

#include <ompl/util/String.h>

#include <ompl/base/Path.h>
#include <ompl/geometric/PathGeometric.h>

unsigned int curDim = 500;

int main(int argc, char **argv)
{
    if (argc > 1)
    {
        curDim = std::atoi(argv[1]);
    }

    auto space(std::make_shared<ompl::base::RealVectorStateSpace>(curDim));
    ompl::base::RealVectorBounds bounds(curDim);
    ompl::geometric::SimpleSetup ss(space);
    ob::SpaceInformationPtr si = ss.getSpaceInformation();
    ob::ProblemDefinitionPtr pdef = ss.getProblemDefinition();
    ompl::base::ScopedState<> start(space), goal(space);

    bounds.setLow(0.);
    bounds.setHigh(1.);
    space->setBounds(bounds);
    ss.setStateValidityChecker(std::make_shared<HyperCubeValidityChecker>(si, curDim));
    for (unsigned int i = 0; i < curDim; ++i)
    {
        start[i] = 0.;
        goal[i] = 1.;
    }
    ss.setStartAndGoalStates(start, goal);

    std::vector<int> admissibleProjection = getHypercubeAdmissibleProjection(curDim);

    ob::PlannerPtr planner = GetMultiLevelPlanner<og::QRRTStar>(admissibleProjection, si, "QRRTStar");
    // ob::PlannerPtr planner = std::make_shared<og::ABITstar>(si);
    // ob::PlannerPtr planner = GetMultiLevelPlanner<og::SPQR>(admissibleProjection, si, "SPQR");
    ss.setPlanner(planner);

    bool solved = ss.solve(10.0);

    double timeToCompute = ss.getLastPlanComputationTime();

    if (solved)
    {
        const ob::ProblemDefinitionPtr pdef = planner->getProblemDefinition();
        std::cout << std::string(80, '-') << std::endl;
        pdef->getSolutionPath()->print(std::cout);
        std::cout << std::string(80, '-') << std::endl;
        OMPL_INFORM("Solved hypercube with %d dimensions after %f seconds.", curDim, timeToCompute);
    }else
    {
        OMPL_ERROR("Failed finding solution after %f seconds.", timeToCompute);
    }

    return 0;
}

