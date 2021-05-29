#include <ompl/multilevel/planners/multimodal/datastructures/PathSpace.h>
#include <ompl/multilevel/planners/multimodal/datastructures/LocalMinimaNode.h>
#include <ompl/multilevel/planners/multimodal/datastructures/LocalMinimaTree.h>
#include <ompl/multilevel/datastructures/PlannerDataVertexAnnotated.h>
#include <ompl/base/Path.h>
#include <boost/foreach.hpp>

#define foreach BOOST_FOREACH

using namespace ompl::multilevel;

PathSpace::PathSpace(BundleSpaceGraph *bundleSpaceGraph) : bundleSpaceGraph_(bundleSpaceGraph)
{
}

void PathSpace::setLocalMinimaTree(LocalMinimaTreePtr localMinimaTree)
{
    localMinimaTree_ = localMinimaTree;
    localMinimaTree_->setEpsilonConvergenceThreshold(epsilonConvergenceThreshold_);
    localMinimaTree_->setNsubtresholdIterations(NsubtresholdIterations_);
}

LocalMinimaTreePtr PathSpace::getLocalMinimaTree()
{
    return localMinimaTree_;
}

PathSpace::~PathSpace()
{
}

void PathSpace::clear()
{
}

ompl::base::PathPtr PathSpace::VerticesToPathPtr(std::vector<BundleSpaceGraph::Vertex> vpath)
{
    const BundleSpaceGraph::Graph &graph = bundleSpaceGraph_->getGraph();

    auto path(std::make_shared<geometric::PathGeometric>(bundleSpaceGraph_->getBundle()));

    for (uint k = 0; k < vpath.size(); k++)
    {
        path->append(graph[vpath.at(k)]->state);
    }
    return path;
}

void PathSpace::removePath(unsigned int k)
{
    int level = bundleSpaceGraph_->getLevel();
    localMinimaTree_->removePath(level, k);
}

void PathSpace::updatePath(unsigned int k, base::PathPtr path, double cost)
{
    int level = bundleSpaceGraph_->getLevel();

    LocalMinimaNode *node = localMinimaTree_->updatePath(path, cost, level, k);

    OMPL_INFORM("Update path %d %s with cost %.2f (%d path(s) on level %d).", 
        k, (node->isConverged()?"[Converged]":""),
        cost, getNumberOfPaths(), level);
}


void PathSpace::updatePath(unsigned int k, std::vector<BundleSpaceGraph::Vertex> vpath, double cost)
{
    int level = bundleSpaceGraph_->getLevel();
    LocalMinimaNode *node = 
      localMinimaTree_->updatePath(VerticesToPathPtr(vpath), cost, level, k);
    node->setVertexPath(vpath);
    OMPL_INFORM("Update path %d with cost %.2f (%d path(s) on level %d).", 
        k, cost, getNumberOfPaths(), level);
}

void PathSpace::addPath(geometric::PathGeometric& gpath, double cost)
{
    int level = bundleSpaceGraph_->getLevel();

    auto path(std::make_shared<geometric::PathGeometric>(bundleSpaceGraph_->getBundle()));
    path->append(gpath);

    localMinimaTree_->addPath(path, cost, level);
    OMPL_INFORM("New path with cost %.2f (%d path(s) on level %d).", cost, getNumberOfPaths(), level);
}

void PathSpace::addPath(std::vector<BundleSpaceGraph::Vertex> vpath, double cost)
{
    int level = bundleSpaceGraph_->getLevel();
    LocalMinimaNode *node = localMinimaTree_->addPath(VerticesToPathPtr(vpath), cost, level);
    node->setVertexPath(vpath);
    OMPL_INFORM("New path with cost %.2f (%d path(s) on level %d).", 
        cost, getNumberOfPaths(), level);
}

void PathSpace::addPath(base::PathPtr path, double cost)
{
    int level = bundleSpaceGraph_->getLevel();
    /*LocalMinimaNode *node = */
    localMinimaTree_->addPath(path, cost, level);
    OMPL_INFORM("New path with cost %.2f (%d path(s) on level %d).", 
        cost, getNumberOfPaths(), level);
}

double PathSpace::getPathCost(unsigned int k) const
{
    return localMinimaTree_->getPathCost(bundleSpaceGraph_->getLevel(), k);
}

unsigned int PathSpace::getBestPathIndex() const
{
  //TODO: use priority queue
  double minCost = std::numeric_limits<double>::infinity();
  unsigned int bestIndex = 0;

  unsigned int L = bundleSpaceGraph_->getLevel();
  unsigned int K = localMinimaTree_->getNumberOfMinima(L);
  for(uint k = 0; k < K; k++)
  {
    const LocalMinimaNode *node = localMinimaTree_->getPath(L, k);
    double cost = node->getCost();
    if(cost < minCost)
    {
      minCost = cost;
      bestIndex = k;
    }
  }
  return bestIndex;
}

double PathSpace::getBestPathCost() const
{
  unsigned int k = getBestPathIndex();
  unsigned int L = bundleSpaceGraph_->getLevel();
  const LocalMinimaNode *node = localMinimaTree_->getPath(L, k);
  return node->getCost();
}

const ompl::base::PathPtr& PathSpace::getBestPathPtr() const
{
  unsigned int k = getBestPathIndex();
  // unsigned int L = bundleSpaceGraph_->getLevel();
  // const LocalMinimaNode *node = localMinimaTree_->getPath(L, k);
  // return node->asPathPtr();
  return getPathPtr(k);
}

bool PathSpace::isPathConverged(unsigned int index) const
{
    const LocalMinimaNode *node = 
      localMinimaTree_->getPath(bundleSpaceGraph_->getLevel(), index);

    return node->isConverged();
}

bool PathSpace::allPathsHaveConverged() const
{
  for(unsigned int index = 0; index < getNumberOfPaths(); index++)
  {
    const LocalMinimaNode *node = 
      localMinimaTree_->getPath(bundleSpaceGraph_->getLevel(), index);

    if(!node->isConverged()) return false;
  }
  return true;
}

ompl::base::PathPtr& PathSpace::getBestPathPtrNonConst() 
{
  unsigned int k = getBestPathIndex();
  return getPathPtrNonConst(k);
}

const ompl::base::PathPtr& PathSpace::getPathPtr(unsigned int k) const
{
    const LocalMinimaNode *node = localMinimaTree_->getPath(bundleSpaceGraph_->getLevel(), k);
    return node->asPathPtr();
}

ompl::base::PathPtr& PathSpace::getPathPtrNonConst(unsigned int k)
{
    LocalMinimaNode *node = localMinimaTree_->getPath(bundleSpaceGraph_->getLevel(), k);
    return node->asPathPtrNonConst();
}

const std::vector<BundleSpaceGraph::Vertex> &PathSpace::getPathVertices(unsigned int k)
{
    const LocalMinimaNode *node = localMinimaTree_->getPath(bundleSpaceGraph_->getLevel(), k);
    return node->asVertices();
}

const std::vector<ompl::base::State*> &PathSpace::getPathStates(unsigned int k)
{
    const LocalMinimaNode *node = localMinimaTree_->getPath(bundleSpaceGraph_->getLevel(), k);
    return node->asStates();
}

std::vector<ompl::base::State*> &PathSpace::getPathStatesNonConst(unsigned int k)
{
    LocalMinimaNode *node = localMinimaTree_->getPath(bundleSpaceGraph_->getLevel(), k);
    return node->asStatesNonConst();
}

unsigned int PathSpace::getNumberOfPaths() const
{
    return localMinimaTree_->getNumberOfMinima(bundleSpaceGraph_->getLevel());
}
