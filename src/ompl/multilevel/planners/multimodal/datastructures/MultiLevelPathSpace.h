#ifndef OMPL_MULTILEVEL_PLANNERS_MULTIMODAL_MULTILEVELMOTIONMULTIMODAL_
#define OMPL_MULTILEVEL_PLANNERS_MULTIMODAL_MULTILEVELMOTIONMULTIMODAL_
#include <ompl/multilevel/datastructures/BundleSpaceGraph.h>
#include <ompl/multilevel/datastructures/BundleSpaceGraphSparse.h>

#include <ompl/multilevel/datastructures/BundleSpaceSequence.h>
#include <ompl/multilevel/planners/multimodal/datastructures/PathSpace.h>
#include <type_traits>
#include <queue>

namespace ompl
{
    namespace multilevel
    {
        OMPL_CLASS_FORWARD(LocalMinimaTree);

        /** \brief Extension Strategy to grow local minima tree.
         *  AUTOMATIC_BREADTH_FIRST: select a uniform random path 
         *    on base space for restriction sampling
         *  AUTOMATIC_DEPTH_FIRST: select best cost path
         *  MANUAL: let user select path
         */
        enum ExtensionStrategy
        {
            AUTOMATIC_BREADTH_FIRST = 0,
            AUTOMATIC_DEPTH_FIRST = 1, 
            MANUAL = 2
        };

        template <class T>
        class MultiLevelPathSpace : public BundleSpaceSequence<T>
        {
            static_assert(std::is_base_of<BundleSpace, T>::value, 
                "Template must inherit from BundleSpace");

            using BaseT = BundleSpaceSequence<T>;

        public:

            const bool DEBUG{false};

            MultiLevelPathSpace(std::vector<base::SpaceInformationPtr> &siVec,
                                std::string type = "MultiLevelPathSpace");
            virtual ~MultiLevelPathSpace() override;

            void getPlannerData(base::PlannerData &data) const override;

            base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;
            void setup() override;
            void clear() override;

            LocalMinimaTreePtr &getLocalMinimaTree();

            bool hasConverged();

            void setExtensionStrategy(ExtensionStrategy);

            ExtensionStrategy getExtensionStrategy();

        protected:
            double pathBias{0.8};  //[0,1]

            T *current{nullptr};

            bool needPreprocessing = false;

            LocalMinimaTreePtr localMinimaTree_;

            ExtensionStrategy extensionStrategy_;

        };
    }
}

#include "MultiLevelPathSpaceImpl.h"
#endif
