#include <ompl/multilevel/planners/explorer/datastructures/PathSpace.h>
#include <ompl/multilevel/datastructures/PlannerDataVertexAnnotated.h>
#include <ompl/multilevel/planners/explorer/datastructures/PathVisibilityChecker.h>
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
}

PathSpace::~PathSpace()
{
}

void PathSpace::clear()
{
}

ompl::base::PathPtr PathSpace::VerticesToPathPtr(VertexPath vpath)
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
}

void PathSpace::updatePath(unsigned int k, base::PathPtr path, double cost)
{
    int level = bundleSpaceGraph_->getLevel();
    // LocalMinimaNode *node = localMinimaTree_->updatePath(path, cost, level, k);
    // node->setVertexPath(vpath);
    OMPL_INFORM("Update path %d with cost %.2f (%d path(s) on level %d).", k, cost, getNumberOfPaths(), level);
}


void PathSpace::updatePath(unsigned int k, VertexPath vpath, double cost)
{
    int level = bundleSpaceGraph_->getLevel();
    LocalMinimaNode *node = localMinimaTree_->updatePath(VerticesToPathPtr(vpath), cost, level, k);
    node->setVertexPath(vpath);
    OMPL_INFORM("Update path %d with cost %.2f (%d path(s) on level %d).", k, cost, getNumberOfPaths(), level);
}

void PathSpace::addPath(geometric::PathGeometric& gpath, double cost)
{
    int level = bundleSpaceGraph_->getLevel();

    auto path(std::make_shared<geometric::PathGeometric>(bundleSpaceGraph_->getBundle()));
    path->append(gpath);

    localMinimaTree_->addPath(path, cost, level);
    OMPL_INFORM("New path with cost %.2f (%d path(s) on level %d).", cost, getNumberOfPaths(), level);
}

void PathSpace::addPath(VertexPath vpath, double cost)
{
    int level = bundleSpaceGraph_->getLevel();
    LocalMinimaNode *node = localMinimaTree_->addPath(VerticesToPathPtr(vpath), cost, level);
    node->setVertexPath(vpath);
    OMPL_INFORM("New path with cost %.2f (%d path(s) on level %d).", cost, getNumberOfPaths(), level);
}

void PathSpace::addPath(base::PathPtr path, double cost)
{
    int level = bundleSpaceGraph_->getLevel();
    /*LocalMinimaNode *node = */
    localMinimaTree_->addPath(path, cost, level);
    OMPL_INFORM("New path with cost %.2f (%d path(s) on level %d).", cost, getNumberOfPaths(), level);
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
  unsigned int L = bundleSpaceGraph_->getLevel();
  const LocalMinimaNode *node = localMinimaTree_->getPath(L, k);
  return node->asPathPtr();
}

ompl::base::PathPtr& PathSpace::getBestPathPtrNonConst() 
{
  unsigned int k = getBestPathIndex();
  unsigned int L = bundleSpaceGraph_->getLevel();
  LocalMinimaNode *node = localMinimaTree_->getPath(L, k);
  return node->asPathPtrNonConst();
}

const ompl::base::PathPtr& PathSpace::getPathPtr(unsigned int k)
{
    const LocalMinimaNode *node = localMinimaTree_->getPath(bundleSpaceGraph_->getLevel(), k);
    return node->asPathPtr();
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

// void PathSpace::getPathIndices(
//     const std::vector<BundleSpaceGraph::Vertex> &vertices,
//     std::vector<int> &idxPath) const
// {
//     if (!bundleSpaceGraph_->hasBaseSpace())
//     {
//         std::reverse(idxPath.begin(), idxPath.end());
//         return;
//     }
//     else
//     {
//       idxPath.push_back(0);
//         // parent->getPathIndices(vertices, idxPath);
//     }
// }

// void PathSpace::getPlannerData(base::PlannerData &data, BundleSpaceGraph* bundleGraph) const
//{
//    BundleSpaceGraph::Graph graph = bundleGraph->getGraph();
//    base::SpaceInformationPtr si = bundleGraph->getBundle();

//    std::vector<int> idxPath;
//    if (criticalPaths_.size() > 0)
//    {
//        OMPL_DEVMSG1("%s has %d solutions.", bundleGraph->getName().c_str(), criticalPaths_.size());

//        for (uint i = 0; i < criticalPaths_.size(); i++)
//        {
//            const std::vector<BundleSpaceGraph::Vertex> vertices = criticalPaths_.at(i);

//            idxPath.clear();
//            getPathIndices(vertices, idxPath);
//            idxPath.push_back(i);

//            std::cout << "[";
//            for (uint k = 0; k < idxPath.size(); k++)
//            {
//                std::cout << idxPath.at(k) << " ";
//            }
//            std::cout << "]" << std::endl;
//            //############################################################################

//            int i1 = vertices.at(0);
//            base::State *s1 = graph[i1]->state;
//            multilevel::PlannerDataVertexAnnotated *p1 =
//              new multilevel::PlannerDataVertexAnnotated(si->cloneState(s1));

//            p1->setLevel(bundleGraph->getLevel());
//            p1->setPath(idxPath);
//            data.addStartVertex(*p1);

//            // si->printState(s1);
//            for (uint k = 0; k < vertices.size() - 1; k++)
//            {
//                base::State *s2 = graph[vertices.at(k+1)]->state;
//                // si->printState(s2);

//                multilevel::PlannerDataVertexAnnotated *p2 =
//                  new multilevel::PlannerDataVertexAnnotated(si->cloneState(s2));
//                p2->setLevel(bundleGraph->getLevel());
//                p2->setPath(idxPath);

//                if (k == vertices.size() - 2)
//                {
//                    data.addGoalVertex(*p2);
//                }
//                else
//                {
//                    data.addVertex(*p2);
//                }
//                data.addEdge(*p1, *p2);

//                p1 = p2;
//            }
//            std::cout << "Cost: " << getPathCost(i) << std::endl;
//        }
//    }
//    foreach (const BundleSpaceGraph::Edge e, boost::edges(graph))
//    {
//        const BundleSpaceGraph::Vertex v1 = boost::source(e, graph);
//        const BundleSpaceGraph::Vertex v2 = boost::target(e, graph);

//        multilevel::PlannerDataVertexAnnotated p1(graph[v1]->state);
//        multilevel::PlannerDataVertexAnnotated p2(graph[v2]->state);
//        p1.setPath(idxPath);
//        p2.setPath(idxPath);
//        data.addEdge(p1, p2);
//    }
//    foreach (const BundleSpaceGraph::Vertex v, boost::vertices(graph))
//    {
//        multilevel::PlannerDataVertexAnnotated p(graph[v]->state);
//        p.setPath(idxPath);
//        data.addVertex(p);
//    }
//}
