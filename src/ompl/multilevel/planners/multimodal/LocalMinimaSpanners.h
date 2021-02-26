#ifndef OMPL_MULTILEVEL_PLANNERS_BUNDLESPACE_LOCALMINIMASPANNERS__
#define OMPL_MULTILEVEL_PLANNERS_BUNDLESPACE_LOCALMINIMASPANNERS__
#include <ompl/multilevel/datastructures/BundleSpaceSequence.h>
#include <ompl/multilevel/planners/multimodal/PathSpaceSparse.h>
#include <ompl/multilevel/planners/multimodal/PathSpaceSparseOptimization.h>
#include <ompl/multilevel/planners/multimodal/datastructures/MultiLevelPathSpace.h>

namespace ompl
{
    namespace multilevel
    {
        using LocalMinimaSpanners = MultiLevelPathSpace<PathSpaceSparse>;
        using MotionExplorer = MultiLevelPathSpace<PathSpaceSparseOptimization>;
    }
}

#endif
