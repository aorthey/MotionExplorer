#ifndef OMPL_MULTILEVEL_PLANNERS_MULTIMODAL_LOCALMINIMANODE_
#define OMPL_MULTILEVEL_PLANNERS_MULTIMODAL_LOCALMINIMANODE_

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
        using VertexPath = std::vector<BundleSpaceGraph::Vertex>;
        using StatesPath = std::vector<base::State *>;

        class LocalMinimaNode
        {
        public:
            LocalMinimaNode(base::SpaceInformationPtr si, base::PathPtr path);
            LocalMinimaNode(base::SpaceInformationPtr si, StatesPath &states);
            LocalMinimaNode(base::SpaceInformationPtr si, VertexPath &vertices);
            ~LocalMinimaNode();

            double getCost() const;
            void setVertexPath(VertexPath &vertices);
            void setPathPtr(base::PathPtr path);

            const StatesPath &asStates() const;
            StatesPath &asStatesNonConst();
            const VertexPath &asVertices() const;
            const base::PathPtr &asPathPtr() const;
            base::PathPtr &asPathPtrNonConst();

            void setCost(double c);
            int getLevel() const;
            void setLevel(int level);
            int getId() const;
            void init();

            bool isConverged() const;

            PathPiecewiseLinear *customRepresentation{nullptr};

            int numberOfIdempotentUpdates_{0};

            void setNsubtresholdIterations(int);

            int getNumberOfCostUpdates();
        private:
            // content
            base::Cost cost_;

            static int id_counter;
            int id_; //required to track history of added/removed paths

            int level_;

            base::SpaceInformationPtr si_;

            bool hasStatesRepresentation{false};
            bool hasVertexRepresentation{false};
            bool hasPathPtrRepresentation{false};

            // different internal representations of path (on-demand)
            VertexPath vpath_;
            StatesPath spath_;
            base::PathPtr path_;

            LocalMinimaNode *parent_{nullptr};

            int numberOfCostUpdates_{0};

            int NsubtresholdIterations_{10};
        };
    }
}
#endif
