#ifndef OMPL_MULTILEVEL_PLANNERS_BUNDLESPACE_TENETIMPL_
#define OMPL_MULTILEVEL_PLANNERS_BUNDLESPACE_TENETIMPL_
#include <ompl/multilevel/datastructures/BundleSpaceGraph.h>
#include <ompl/datastructures/PDF.h>

namespace ompl
{
    namespace multilevel
    {
        class TENETImpl : public ompl::multilevel::BundleSpaceGraph
        {
            using BaseT = BundleSpaceGraph;

        public:
            TENETImpl(const ompl::base::SpaceInformationPtr &si, BundleSpace *parent_);
            virtual ~TENETImpl() override;

            /** \brief One iteration of RRT with adjusted sampling function */
            virtual void grow() override;

        };
    }  // namespace multilevel
}  // namespace ompl

#endif
