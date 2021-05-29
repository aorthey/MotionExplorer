#include <ompl/multilevel/datastructures/PlannerDataVertexAnnotated.h>
#include <ompl/multilevel/datastructures/BundleSpace.h>
#include <ompl/multilevel/planners/multimodal/datastructures/PathSpace.h>
#include <ompl/multilevel/planners/multimodal/datastructures/LocalMinimaTree.h>
#include <ompl/multilevel/planners/multimodal/datastructures/LocalMinimaNode.h>
#include <ompl/multilevel/planners/multimodal/PathSpaceSparse.h>

#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/spaces/SO3StateSpace.h>
#include <ompl/util/Time.h>
#include <ompl/util/Exception.h>
#include <queue>

using namespace ompl::multilevel;

template <class T>
ompl::multilevel::MultiLevelPathSpace<T>::MultiLevelPathSpace(std::vector<base::SpaceInformationPtr> &siVec,
                                                              std::string type)
  : BaseT(siVec, type)
{
    current = this->bundleSpaces_.front();
    localMinimaTree_ = std::make_shared<LocalMinimaTree>(siVec);

    // connect all quotient spaces to local minima tree
    for (uint k = 0; k < this->bundleSpaces_.size(); k++)
    {
        PathSpace *Pk = static_cast<PathSpace *>(this->bundleSpaces_.at(k));
        Pk->setLocalMinimaTree(localMinimaTree_);
    }

    setExtensionStrategy(ExtensionStrategy::MANUAL);
    setExtensionStrategy(ExtensionStrategy::AUTOMATIC_DEPTH_FIRST);
}

template <class T>
MultiLevelPathSpace<T>::~MultiLevelPathSpace()
{
}

template<class T>
void MultiLevelPathSpace<T>::setExtensionStrategy(ExtensionStrategy extensionStrategy)
{
    extensionStrategy_ = extensionStrategy;
}

template<class T>
ExtensionStrategy MultiLevelPathSpace<T>::getExtensionStrategy()
{
    return extensionStrategy_;
}


template <class T>
void ompl::multilevel::MultiLevelPathSpace<T>::setup()
{
    BaseT::setup();
}

template <class T>
void ompl::multilevel::MultiLevelPathSpace<T>::clear()
{
    BaseT::clear();
    localMinimaTree_->clear();
    current = this->bundleSpaces_.front();
    OMPL_INFORM("Cleared multilevel path space structure.");
}
template <class T>
LocalMinimaTreePtr &MultiLevelPathSpace<T>::getLocalMinimaTree()
{
    return localMinimaTree_;
}

template <class T>
ompl::base::PlannerStatus MultiLevelPathSpace<T>::solve(const ompl::base::PlannerTerminationCondition &ptc)
{
    ompl::msg::setLogLevel(ompl::msg::LOG_DEV2);

    //Choice of bundle space to grow depends on extensionstrategy
    BundleSpace *jBundle;

    //NOTE:
    //manual extension: take a specific minima and only expand in its path
    //restriction
    //
    //automatic extension (depth first): For each level, pick the best-cost path
    //and sample in its path restriction.
    //
    //automatic extension (breadth first): For each level, try to sample until
    //convergence, then iterate through all minima by sampling their path
    //restriction.
    if(extensionStrategy_ == ExtensionStrategy::MANUAL)
    {
        std::vector<int> selectedLocalMinimum = localMinimaTree_->getSelectedPathIndex();
        uint K = selectedLocalMinimum.size();

        if (K >= this->bundleSpaces_.size())
        {
            K = K - 1;
        }

        // find lowest dimensional QS which has a solution. Then take the next QS to
        // expand
        while (K > 0)
        {
            if (localMinimaTree_->getNumberOfMinima(K - 1) > 0)
            {
                break;
            }
            else
            {
                K = K - 1;
            }
        }

        std::cout << "Growing on " << K << "-th space." << std::endl;

        // Check which
        jBundle = static_cast<BundleSpace *>(this->bundleSpaces_.at(K));
    }else if(extensionStrategy_ == ExtensionStrategy::AUTOMATIC_BREADTH_FIRST)
    {
        OMPL_ERROR("Extension Strategy not yet implemented.");
        throw "NYI";
    }else if(extensionStrategy_ == ExtensionStrategy::AUTOMATIC_DEPTH_FIRST)
    {
        //iterate through all levels which contain a minimum path and set the
        //best-cost path as the selected minimum. Then select one level at random
        //with a large bias towards the highest non-minimum path containing level. 
        //Then exand the selected level: do random sampling with bias
        //towards the principal path restriction. 

        uint K = this->bundleSpaces_.size();

        while (K > 0)
        {
            if (localMinimaTree_->getNumberOfMinima(K - 1) > 0)
            {
                break;
            }
            else
            {
                K = K - 1;
            }
        }

        std::vector<int> principalPathIndices;

        for(uint k = 0; k < K; k++)
        {
            std::vector<LocalMinimaNode *> paths = localMinimaTree_->getPaths(k);
            double d_best = dInf;
            int j_best = 0;
            for(uint j = 0; j < paths.size(); j++)
            {
              LocalMinimaNode *pj = paths.at(j);
              double dj = pj->getCost();
              if(dj < dInf)
              {
                d_best = dj;
                j_best = j;
              }
            }
            principalPathIndices.push_back(j_best);
        }

        //setting best cost paths
        // localMinimaTree_->setSelectedPathIndex(principalPathsIndices);

        jBundle = static_cast<BundleSpace *>(this->bundleSpaces_.at(K));

    }else{
        OMPL_ERROR("Extension Strategy is unknown: %d", extensionStrategy_);
        throw "NYI";
    }

    uint ctr = 0;

    // grow at least PTC, then grow until solution (that way we do not stop after
    // finding just one single path)
    while (!ptc())
    {
        jBundle->grow();
        if (jBundle->isInfeasible())
        {
            OMPL_DEBUG("Infeasibility detected (level %d).", jBundle->getLevel());
            return ompl::base::PlannerStatus::INFEASIBLE;
        }

        if (jBundle->hasConverged())
        {
            OMPL_DEBUG("Converged (level %d).", jBundle->getLevel());
            return ompl::base::PlannerStatus::APPROXIMATE_SOLUTION;
        }
        ctr++;
    }
    return base::PlannerStatus::TIMEOUT;
}

template <class T>
bool MultiLevelPathSpace<T>::hasConverged()
{
    for(uint k = 0; k < this->bundleSpaces_.size(); k++)
    {
        if(!this->bundleSpaces_.at(k)->hasConverged()) return false;
    }
    return true;
}

template <class T>
void MultiLevelPathSpace<T>::getPlannerData(base::PlannerData &data) const
{
    // TODO: just call BaseT (better: remove completely)
    unsigned int Nvertices = data.numVertices();
    if (Nvertices > 0)
    {
        OMPL_ERROR("PlannerData has %d vertices.", Nvertices);
        throw ompl::Exception("cannot get planner data if plannerdata is already populated");
    }

    unsigned int K = this->bundleSpaces_.size();
    std::vector<uint> countVerticesPerBundleSpace;

    BundleSpace *Qlast = this->bundleSpaces_.back();
    for (unsigned int k = 0; k < K; k++)
    {
        BundleSpaceGraph *Qk = static_cast<BundleSpaceGraph *>(this->bundleSpaces_.at(k));

        Qk->getPlannerData(data);

        for (unsigned int vidx = Nvertices; vidx < data.numVertices(); vidx++)
        {
            multilevel::PlannerDataVertexAnnotated &v =
                *static_cast<multilevel::PlannerDataVertexAnnotated *>(&data.getVertex(vidx));
            v.setLevel(k);
            v.setMaxLevel(K);

            base::State *s_lift = BaseT::getTotalState(k, v.getBaseState());
            v.setTotalState(s_lift, Qlast->getBundle());
        }
        countVerticesPerBundleSpace.push_back(data.numVertices() - Nvertices);
        Nvertices = data.numVertices();
    }
}
