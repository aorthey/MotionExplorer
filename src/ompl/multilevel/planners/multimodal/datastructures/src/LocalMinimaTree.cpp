#include "util.h"
#include <ompl/multilevel/planners/multimodal/datastructures/LocalMinimaTree.h>
#include <ompl/multilevel/planners/multimodal/datastructures/LocalMinimaNode.h>
#include <ompl/multilevel/planners/multimodal/datastructures/PathSpaceMetrics.h>
#include <ompl/util/Console.h>
#include <ompl/geometric/PathSimplifier.h>
#include <thread>

using namespace ompl::multilevel;

LocalMinimaTree::LocalMinimaTree(std::vector<base::SpaceInformationPtr> siVec) : siVec_(siVec)
{
    OMPL_DEBUG("Number of threads supported by implementation: %d", 
        std::thread::hardware_concurrency());
    for (uint k = 0; k < siVec.size(); k++)
    {
        std::vector<LocalMinimaNode *> kthLevel;
        tree_.push_back(kthLevel);
    }
}

LocalMinimaTree::~LocalMinimaTree()
{
}

void LocalMinimaTree::clear()
{
    hasChanged_ = false;
    selectedMinimum_.clear();

    for (uint k = 0; k < tree_.size(); k++)
    {
        std::vector<LocalMinimaNode *> &treeLevelk = tree_.at(k);
        for (uint j = 0; j < treeLevelk.size(); j++)
        {
            LocalMinimaNode *node = treeLevelk.at(j);
            delete node;
            treeLevelk.at(j) = nullptr;
        }
        treeLevelk.clear();
    }

    numberOfMinima_ = 0;
    levels_ = 0;
}

unsigned int LocalMinimaTree::getNumberOfMinima(unsigned int level) const
{
    return tree_.at(level).size();
}

unsigned int LocalMinimaTree::getNumberOfMinima() const
{
    return numberOfMinima_;
}

unsigned int LocalMinimaTree::getNumberOfLevel() const
{
    return tree_.size();
}

std::vector<int> LocalMinimaTree::getSelectedPathIndex() const
{
    return selectedMinimum_;
}

void LocalMinimaTree::setSelectedPathIndex(std::vector<int> selectedMinimum)
{
    std::lock_guard<std::recursive_mutex> guard(getLock());
    selectedMinimum_ = selectedMinimum;
}

std::recursive_mutex& LocalMinimaTree::getLock()
{
    return lock_;
}

unsigned int LocalMinimaTree::getNumberOfLevelContainingMinima() const
{
    unsigned int ctr = 0;

    while (ctr <= (tree_.size() - 1) && tree_.at(ctr).size() > 0)
    {
        ctr++;
    }
    return ctr;
}

bool LocalMinimaTree::hasChanged()
{
    std::lock_guard<std::recursive_mutex> guard(getLock());
    if (hasChanged_)
    {
        hasChanged_ = false;
        return true;
    }
    else
    {
        return false;
    }
}

bool LocalMinimaTree::isConverged() const
{
    const LocalMinimaNode *node = getSelectedPath();
    if(node==nullptr || !node->isConverged()) return false;

    std::vector<LocalMinimaNode *> nodes = getSelectedPathSiblings();
    for(uint k = 0; k < nodes.size(); k++)
    {
      LocalMinimaNode *node = nodes.at(k);
      if(!node->isConverged()) return false;
    }
    return true;
}

LocalMinimaNode *LocalMinimaTree::updatePath(
    base::PathPtr path, double cost, int level, int index)
{
    std::lock_guard<std::recursive_mutex> guard(getLock());
    sanityCheckLevelIndex(level, index);

    LocalMinimaNode *node = tree_.at(level).at(index);
    node->setPathPtr(path);
    node->setLevel(level);
    // node->update();

    double costOld = node->getCost();
    node->setCost(cost);

    double error = fabs(cost - costOld);
    if(error > epsilonConvergenceThreshold_)
    {
        node->numberOfIdempotentUpdates_ = 0;
    }else
    {
        node->numberOfIdempotentUpdates_++;
    }

    hasChanged_ = true;

    if ((int)selectedMinimum_.size() == level - 1)
    {
        selectedMinimum_.back() = index;
    }
    return node;
}

LocalMinimaNode *LocalMinimaTree::addPath(
    base::PathPtr path, double cost, int level)
{
    std::lock_guard<std::recursive_mutex> guard(getLock());

    // base::OptimizationObjectivePtr obj =
    //   std::make_shared<ompl::base::PathLengthOptimizationObjective>(siVec_.at(level));
    // geometric::PathSimplifier shortcutter(siVec_.at(level), base::GoalPtr(), obj);

    // geometric::PathGeometricPtr gpath = std::dynamic_pointer_cast<geometric::PathGeometric>(path);
    // if (gpath != nullptr)
    // {
    //     // gpath->interpolate();
    // }

    LocalMinimaNode *node = new LocalMinimaNode(siVec_.at(level), path);
    node->setLevel(level);
    node->setCost(cost);
    node->setNsubtresholdIterations(NsubtresholdIterations_);
    // node->update();

    tree_.at(level).push_back(node);
    numberOfMinima_++;
    hasChanged_ = true;

    setSelectedMinimumExpand();
    selectedMinimum_.back() = tree_.at(level).size() - 1;
    return node;
}

void LocalMinimaTree::setEpsilonConvergenceThreshold(double epsilon)
{
    epsilonConvergenceThreshold_ = epsilon;
}

void LocalMinimaTree::setNsubtresholdIterations(int N)
{
    NsubtresholdIterations_ = N;
}

void LocalMinimaTree::removePath(int level, int index)
{
    std::lock_guard<std::recursive_mutex> guard(getLock());

    LocalMinimaNode *node = tree_.at(level).at(index);
    if(node->customRepresentation) delete node->customRepresentation;
    if(node) delete node;

    tree_.at(level).erase(tree_.at(level).begin() + index);
    numberOfMinima_--;
    hasChanged_ = true;

    setSelectedMinimumNext();
    setSelectedMinimumPrev();
}

LocalMinimaNode *LocalMinimaTree::getPath(int level, int index) const
{
    sanityCheckLevelIndex(level, index);
    return tree_.at(level).at(index);
}

LocalMinimaNode *LocalMinimaTree::getSelectedPath() const
{
    int level = selectedMinimum_.size() - 1;
    if (level < 0)
        return nullptr;

    int index = selectedMinimum_.back();
    return getPath(level, index);
}

std::vector<LocalMinimaNode *> LocalMinimaTree::getPaths(int level) const
{
    std::vector<LocalMinimaNode *> nodeVector;
    for (uint index = 0; index < getNumberOfMinima(level); index++)
    {
        LocalMinimaNode *nodek = getPath(level, index);
        nodeVector.push_back(nodek);
    }
    return nodeVector;
}

std::vector<LocalMinimaNode *> LocalMinimaTree::getSelectedPathSiblings() const
{
    std::vector<LocalMinimaNode *> nodeVector;

    int level = selectedMinimum_.size() - 1;
    if (level < 0)
        return nodeVector;

    int indexSelectedPath = selectedMinimum_.back();

    for (uint k = 0; k < getNumberOfMinima(level); k++)
    {
        if ((int)k == indexSelectedPath)
            continue;
        LocalMinimaNode *nodek = getPath(level, k);
        nodeVector.push_back(nodek);
    }
    return nodeVector;
}

void LocalMinimaTree::sanityCheckLevelIndex(int level, int index) const
{
    if (level < 0 || level > (int)getNumberOfLevel() - 1)
    {
        OMPL_ERROR("Tried getting path cost for non existing path.");
        throw ompl::Exception("Nonexisting");
    }
    if (index < 0 || index > (int)getNumberOfMinima(level) - 1)
    {
        OMPL_ERROR("Tried getting path cost for non existing path.");
        throw ompl::Exception("Nonexisting");
    }
}

double LocalMinimaTree::getPathCost(int level, int index) const
{
    sanityCheckLevelIndex(level, index);
    LocalMinimaNode *node = tree_.at(level).at(index);
    return node->getCost();
}

void LocalMinimaTree::printSelectedMinimum()
{
    std::lock_guard<std::recursive_mutex> guard(getLock());
    if (selectedMinimum_.size() > 0)
    {
        int level = selectedMinimum_.size() - 1;
        int maxMinima = getNumberOfMinima(selectedMinimum_.size() - 1);
        int curMinimum = selectedMinimum_.back();
        OMPL_DEVMSG1("Selected local minimum %d/%d (level %d, %d updates, cost %.2f%s)", 
          selectedMinimum_.back() + 1, 
          maxMinima, 
          level,
          tree_.at(level).at(curMinimum)->getNumberOfCostUpdates(),
          tree_.at(level).at(curMinimum)->getCost(),
          (tree_.at(level).at(curMinimum)->isConverged()?
           ", [converged]":""));
        ompl::msg::LogLevel logLevel = ompl::msg::getLogLevel();
        if(logLevel <= ompl::msg::LOG_DEV2)
        {
          for(int k = 0; k < maxMinima; k++)
          {
            if(k != curMinimum)
            {
              LocalMinimaNode *nk = tree_.at(level).at(k);
              LocalMinimaNode *ncur = tree_.at(level).at(curMinimum);
              const std::vector<base::State*> &sk = nk->asStates();
              const std::vector<base::State*> &scur = ncur->asStates();
              double dist = pathMetric_MaxMin(sk, scur, siVec_.at(level));

              OMPL_DEVMSG2(" -- Distance to path %d/%d (cost %.2f%s): %.2f", 
                k+1,
                maxMinima, 
                nk->getCost(),
                (nk->isConverged()?"[c]":""),
                dist);
            }
          }
        }
    }
}

void LocalMinimaTree::setSelectedMinimumPrev()
{
    std::lock_guard<std::recursive_mutex> guard(getLock());

    if (selectedMinimum_.size() > 0)
    {
        int maxMinima = getNumberOfMinima(selectedMinimum_.size() - 1);
        if (maxMinima > 0)
        {
            if (selectedMinimum_.back() > 0)
            {
                selectedMinimum_.back()--;
            }
            else
            {
                selectedMinimum_.back() = maxMinima - 1;
            }
        }
    }
    hasChanged_ = true;
}


void LocalMinimaTree::setSelectedMinimumNext()
{
    std::lock_guard<std::recursive_mutex> guard(getLock());

    if (selectedMinimum_.size() > 0)
    {
        int maxMinima = getNumberOfMinima(selectedMinimum_.size() - 1);
        if (maxMinima > 0)
        {
            if (selectedMinimum_.back() < maxMinima - 1)
            {
                selectedMinimum_.back()++;
            }else{
                selectedMinimum_.back() = 0;
            }
        }
    }
    hasChanged_ = true;
}

void LocalMinimaTree::setSelectedMinimumCollapse()
{
    std::lock_guard<std::recursive_mutex> guard(getLock());

    if (selectedMinimum_.size() > 0)
    {
        selectedMinimum_.erase(selectedMinimum_.end() - 1);
    }
    hasChanged_ = true;
}

void LocalMinimaTree::setSelectedMinimumExpand()
{
    std::lock_guard<std::recursive_mutex> guard(getLock());

    unsigned int maxLevel = getNumberOfLevelContainingMinima();
    if (selectedMinimum_.size() < maxLevel)
    {
        selectedMinimum_.push_back(0);
    }
    hasChanged_ = true;
}

void LocalMinimaTree::setSelectedMinimumExpandFull()
{
    std::lock_guard<std::recursive_mutex> guard(getLock());

    unsigned int maxLevel = getNumberOfLevelContainingMinima();
    while (selectedMinimum_.size() < maxLevel)
    {
        selectedMinimum_.push_back(0);
    }
    hasChanged_ = true;
}

const std::vector<ompl::base::State *>&
LocalMinimaTree::getSelectedMinimumAsStateVector(int level) const
{
    int selectedMinimumOnLevel = selectedMinimum_.at(level);
    LocalMinimaNode *node = tree_.at(level).at(selectedMinimumOnLevel);
    return node->asStates();
}
