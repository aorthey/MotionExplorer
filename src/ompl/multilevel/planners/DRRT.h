#ifndef OMPL_MULTILEVEL_PLANNERS_BUNDLESPACE_DRRT_
#define OMPL_MULTILEVEL_PLANNERS_BUNDLESPACE_DRRT_
#include <ompl/multilevel/datastructures/BundleSpaceSequence.h>
#include <ompl/multilevel/planners/DRRTImpl.h>

namespace ompl
{
    namespace multilevel
    {
        using DRRT = BundleSpaceSequence<DRRTImpl>;
    }
}

#endif



