#ifndef OMPL_MULTILEVEL_PLANNERS_BundleSpace_PathSpaceSparse_
#define OMPL_MULTILEVEL_PLANNERS_BundleSpace_PathSpaceSparse_
#include <ompl/multilevel/planners/sparse/SMLRImpl.h>
#include <ompl/multilevel/planners/multimodal/datastructures/PathSpace.h>
#include <ompl/geometric/PathGeometric.h>

namespace ompl
{
    namespace base
    {
        OMPL_CLASS_FORWARD(OptimizationObjective);
        OMPL_CLASS_FORWARD(Path);
    }
    namespace geometric
    {
        OMPL_CLASS_FORWARD(PathSimplifier);
    }
    namespace multilevel
    {
        class PathSpaceSparse : 
          public ompl::multilevel::PathSpace, 
          public ompl::multilevel::SMLRImpl
        {
            using BaseT = SMLRImpl;

        public:

            PathSpaceSparse(const ompl::base::SpaceInformationPtr &si, BundleSpace *parent_);

            virtual ~PathSpaceSparse() override;

            virtual bool hasConverged() override;

            virtual void grow() override;

            virtual void setup() override;

            virtual base::PathPtr& getSolutionPathByReference() override;

            virtual const std::pair<BundleSpaceGraph::Edge, bool> 
              addEdge(const Vertex a, const Vertex b) override;

            void checkPath(const Vertex v, const Vertex vStart, const Vertex vGoal);

            base::PathPtr constructPath(const Vertex v, const Vertex vStart, const Vertex vGoal);

            bool arePathsEquivalent( ompl::base::PathPtr path1, 
                ompl::base::PathPtr path2);

            unsigned int getRandomNonConvergedPathIndex();

        protected:
            double bestCost_{std::numeric_limits<double>::infinity()};

            double epsilonPathEquivalence_{0.0};

            void optimizePath(geometric::PathGeometric&);

            double getPathCost(const geometric::PathGeometric&);

            const Vertex 
            getNearestGoalVertex(const std::pair<BundleSpaceGraph::Edge, bool> &edge);
            const Vertex 
            getNearestStartVertex(const std::pair<BundleSpaceGraph::Edge, bool> &edge);
        };
    }  // namespace multilevel
}  // namespace ompl

#endif
