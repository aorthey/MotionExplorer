#ifndef OMPL_MULTILEVEL_PLANNERS_BUNDLESPACE_TENET_
#define OMPL_MULTILEVEL_PLANNERS_BUNDLESPACE_TENET_
#include <ompl/multilevel/datastructures/BundleSpaceSequence.h>
#include <ompl/multilevel/planners/TENETImpl.h>

namespace ompl
{
    namespace multilevel
    {
        using TENET = BundleSpaceSequence<TENETImpl>;
    }
}

#endif



