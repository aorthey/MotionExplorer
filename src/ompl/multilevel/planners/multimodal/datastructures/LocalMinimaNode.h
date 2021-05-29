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

            double getCost() const
            {
                return cost_.value();
            }

            void setVertexPath(VertexPath &vertices)
            {
                vpath_ = vertices;
                hasVertexRepresentation = true;
            }

            void setPathPtr(base::PathPtr path)
            {
                path_ = path;
                hasPathPtrRepresentation = true;

                geometric::PathGeometricPtr gpath = 
                  std::static_pointer_cast<geometric::PathGeometric>(path);
                spath_ = gpath->getStates();
                hasStatesRepresentation = true;
            }

            const StatesPath &asStates() const;
            StatesPath &asStatesNonConst();
            const VertexPath &asVertices() const;
            const base::PathPtr &asPathPtr() const;
            base::PathPtr &asPathPtrNonConst();

            void setCost(double c)
            {
                cost_ = base::Cost(c);
            }

            int getLevel() const
            {
                return level_;
            }

            void setLevel(int level)
            {
                level_ = level;
            }

            int getId() const
            {
              return id_;
            }
            //XML routines
            void init();

            bool isConverged() const;

            // void *customRepresentation{nullptr};
            PathPiecewiseLinear *customRepresentation{nullptr};

            int numberOfIdempotentUpdates_{0};

            void setNsubtresholdIterations(int);

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

            int NsubtresholdIterations_{10};
        };
    }
}
#endif
