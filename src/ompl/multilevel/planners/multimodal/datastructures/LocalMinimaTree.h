#ifndef OMPL_MULTILEVEL_PLANNERS_MULTIMODAL_LOCALMINIMATREE_
#define OMPL_MULTILEVEL_PLANNERS_MULTIMODAL_LOCALMINIMATREE_

#include "elements/path_pwl.h"
#include <ompl/util/ClassForward.h>
#include <ompl/base/State.h>
#include <ompl/base/Cost.h>
#include <ompl/multilevel/datastructures/BundleSpaceGraph.h>
#include <vector>
#include <mutex>

namespace ompl
{
    namespace base
    {
        OMPL_CLASS_FORWARD(SpaceInformation);
        OMPL_CLASS_FORWARD(StateSpace);
        OMPL_CLASS_FORWARD(Path);
    }
    namespace geometric
    {
        OMPL_CLASS_FORWARD(PathSimplifier);
        OMPL_CLASS_FORWARD(PathGeometric);
    }
    namespace multilevel
    {
        OMPL_CLASS_FORWARD(LocalMinimaNode);

        /** \brief Local minima tree as database of local minima plus connections between minima */
        class LocalMinimaTree
        {
        public:
            LocalMinimaTree() = delete;
            LocalMinimaTree(const LocalMinimaTree &) = delete;
            LocalMinimaTree(std::vector<base::SpaceInformationPtr>);
            ~LocalMinimaTree();

            void clear();

            bool isConverged() const;

            void setEpsilonConvergenceThreshold(double);
            void setNsubtresholdIterations(int);

            unsigned int getNumberOfMinima(unsigned int level) const;
            unsigned int getNumberOfMinima() const;
            unsigned int getNumberOfLevel() const;
            unsigned int getNumberOfLevelContainingMinima() const;

            const std::vector<base::State *> 
              &getSelectedMinimumAsStateVector(int level) const;

            /* Browser-like functionalities to select minimum (can be
             * mapped to hjkl, arrow keys or mouse buttons) */
            void setSelectedMinimumPrev();
            void setSelectedMinimumNext();
            void setSelectedMinimumCollapse();
            void setSelectedMinimumExpand();
            void setSelectedMinimumExpandFull();

            void printSelectedMinimum();

            LocalMinimaNode *getPath(int level, int index) const;
            double getPathCost(int level, int index) const;

            LocalMinimaNode *getSelectedPath() const;
            std::vector<int> getSelectedPathIndex() const;
            void setSelectedPathIndex(std::vector<int> index);
            std::vector<LocalMinimaNode *> getSelectedPathSiblings() const;

            /* \brief Return all paths on a given level */
            std::vector<LocalMinimaNode *> getPaths(int level) const;

            void sanityCheckLevelIndex(int level, int index) const;

            LocalMinimaNode *addPath(base::PathPtr path, double cost, int level);
            LocalMinimaNode *updatePath(base::PathPtr path, double cost, int level, int index);
            void removePath(int level, int index);

            std::recursive_mutex& getLock();

            bool hasChanged();

        protected:

            std::recursive_mutex lock_;

            bool hasChanged_{false};

            // needed to convert between representations of tree
            std::vector<base::SpaceInformationPtr> siVec_;
            std::vector<int> selectedMinimum_;
            std::vector<std::vector<LocalMinimaNode *>> tree_;

            int numberOfMinima_{0};
            int levels_{0};

            double epsilonConvergenceThreshold_{1e-2};
            int NsubtresholdIterations_{10};

        };
    }
}
#endif
