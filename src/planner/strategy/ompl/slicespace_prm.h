#pragma once

#include "slicespace.h"

#include <ompl/geometric/planners/PlannerIncludes.h>
#include <ompl/datastructures/NearestNeighbors.h>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/pending/disjoint_sets.hpp>
#include <utility>
#include <vector>
#include <map>

namespace ob = ompl::base;
namespace og = ompl::geometric;


namespace ompl
{
    namespace base
    {
        OMPL_CLASS_FORWARD(OptimizationObjective);
    }

    namespace geometric
    {

        class SliceSpacePRM : public base::Planner
        {
        public:
            SliceSpacePRM(const base::SpaceInformationPtr &si0, const ob::SpaceInformationPtr &si1);
            ~SliceSpacePRM() override;
            void setProblemDefinition(const base::ProblemDefinitionPtr &pdef) override;
            void getPlannerData(base::PlannerData &data) const override;
            base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;

            void clear() override;
            void setup() override;

        protected:

            SliceSpace *S_0;
            SliceSpace *S_1;
            //std::vector<SliceSpace*> Q;

            ob::SpaceInformationPtr si_level0;
            ob::SpaceInformationPtr si_level1;
        };
    }
}

