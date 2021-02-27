#ifndef OMPL_MULTILEVEL_PLANNERS_BundleSpace_PathSpaceSparse_
#define OMPL_MULTILEVEL_PLANNERS_BundleSpace_PathSpaceSparse_
#include <ompl/multilevel/planners/sparse/SMLRImpl.h>
#include <ompl/multilevel/planners/multimodal/datastructures/PathSpace.h>
#include <ompl/datastructures/PDF.h>
#include <ompl/geometric/PathGeometric.h>

namespace ompl
{
    namespace base
    {
        OMPL_CLASS_FORWARD(OptimizationObjective);
    }
    namespace geometric
    {
        OMPL_CLASS_FORWARD(PathSimplifier);
    }
    namespace multilevel
    {
        OMPL_CLASS_FORWARD(PathVisibilityChecker);

        class PathSpaceSparse : 
          public ompl::multilevel::PathSpace, 
          public ompl::multilevel::SMLRImpl
        {
            using BaseT = SMLRImpl;

        public:

            PathSpaceSparse(const ompl::base::SpaceInformationPtr &si, BundleSpace *parent_);

            virtual ~PathSpaceSparse() override;

            // virtual void grow() override;

            virtual void setup() override;

            virtual ompl::base::PathPtr& getSolutionPathByReference() override;

            virtual const std::pair<BundleSpaceGraph::Edge, bool> 
              addEdge(const Vertex a, const Vertex b) override;

            void checkPath(const Vertex v, const Vertex vStart, const Vertex vGoal);


        protected:
            double bestCost_{std::numeric_limits<double>::infinity()};

            PathVisibilityChecker *pathVisibilityChecker_{nullptr};

            void optimizePath(geometric::PathGeometric&);
        };
    }  // namespace multilevel
}  // namespace ompl

#endif
