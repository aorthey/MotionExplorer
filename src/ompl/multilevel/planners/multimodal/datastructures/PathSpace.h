#pragma once
#include <ompl/multilevel/datastructures/BundleSpaceGraph.h>
#include <ompl/base/PlannerData.h>
#include <ompl/geometric/PathGeometric.h>

namespace ompl
{
    namespace base
    {
        OMPL_CLASS_FORWARD(Path);
    }
    namespace multilevel
    {
        OMPL_CLASS_FORWARD(LocalMinimaTree);

        class PathSpace
        {
        public:
            PathSpace() = delete;
            PathSpace(BundleSpaceGraph *);
            ~PathSpace();

            void setLocalMinimaTree(LocalMinimaTreePtr);

            LocalMinimaTreePtr getLocalMinimaTree();

            virtual unsigned int getNumberOfPaths() const;

            base::PathPtr VerticesToPathPtr(std::vector<BundleSpaceGraph::Vertex>);

            void updatePath(unsigned int k, std::vector<BundleSpaceGraph::Vertex> p, double cost);

            void updatePath(unsigned int k, base::PathPtr path, double cost);

            void removePath(unsigned int k);

            void addPath(std::vector<BundleSpaceGraph::Vertex> p, double cost);

            void addPath(base::PathPtr path, double cost);

            void addPath(geometric::PathGeometric&, double cost);

            double getPathCost(unsigned int k) const;

            void clear();

            void setup();

            bool isPathConverged(unsigned int k) const;

            bool allPathsHaveConverged() const;

            unsigned int getBestPathIndex() const;
            double getBestPathCost() const;
            const base::PathPtr& getBestPathPtr() const;
            base::PathPtr& getBestPathPtrNonConst();
            const base::PathPtr& getPathPtr(unsigned int k) const;
            base::PathPtr& getPathPtrNonConst(unsigned int k);

            const std::vector<BundleSpaceGraph::Vertex> &getPathVertices(unsigned int k);
            const std::vector<base::State*> &getPathStates(unsigned int k);
            std::vector<base::State*> &getPathStatesNonConst(unsigned int k);

        protected:
            BundleSpaceGraph *bundleSpaceGraph_;

            LocalMinimaTreePtr localMinimaTree_;

            double epsilonConvergenceThreshold_{1e-2};

            int NsubtresholdIterations_{10};
        };
    }
}
