#include <ompl/multilevel/planners/multimodal/PathSpaceSparse.h>
#include <ompl/multilevel/planners/multimodal/datastructures/PathSpaceMetrics.h>
#include <ompl/multilevel/planners/multimodal/datastructures/LocalMinimaTree.h>
#include <ompl/multilevel/datastructures/graphsampler/GraphSampler.h>
#include <ompl/multilevel/datastructures/PlannerDataVertexAnnotated.h>

#include <ompl/base/Path.h>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/objectives/MaximizeMinClearanceObjective.h>

#include <ompl/geometric/PathSimplifier.h>
#include <ompl/tools/config/SelfConfig.h>
#include <ompl/datastructures/NearestNeighbors.h>
#include <ompl/util/Exception.h>

#include <boost/math/constants/constants.hpp>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

double dInf = std::numeric_limits<double>::infinity();

using namespace ompl::multilevel;

//The idea here is to highjack the "addEdge" method of sparse roadmap spanners. 
//Whenever this function is called, we compute a candidate path which we then
//optimize and potentially add to our local minima tree.

PathSpaceSparse::PathSpaceSparse(const base::SpaceInformationPtr &si, BundleSpace *parent_)
  : PathSpace(this), BaseT(si, parent_)
{
    setName("PathSpaceSparse" + std::to_string(id_));

    //NOTES on parameters:
    //# Sparse Roadmap Parameters
    //*sparsedeltafraction_:
    //  * On 2D scenarios, 0.1 seems appropriate, because otherwise the graph
    //  is too sparse and narrow passages might not be found reliably. Original
    //  values were 0.25 (Dobson and Bekris, 2014) and 0.15 (Orthey and
    //  Toussaint, WAFR, 2020).
    //* maxfailures_:
    //  * seems relatively robust, original value is 5000 (Dobson and Bekris,
    //  2014). But a value of 1000 gives an estimated probability of free space sampled of
    //  1 - 1/1000 = 0.999, which is already good enough for most scenarios.
    //
    //# Optimizer Parameters
    //* epsilonpathequivalence_:
    //  * this parameter details when the distance between paths is small enough
    //  to consider them to be equivalent. Here, we need to set
    //  it relatively high, because the OMPL optimizer we use often gets stuck
    //  near to a local minima mode. However, this should be elevated once we
    //  switch to more powerful optimizers.
    //* epsilonconvergencethreshold_:
    //  * specifies the amount of change between optimizer iterations to
    //  consider the optimizer to have converged. This parameter needs to be
    //  combined with the Nsubtresholditerations_ parameter because we often
    //  have optimizer iterations which changes cost minimally, but which still
    //  have not yet converged.
    //* Nsubtresholditerations_:
    //  * Specifies after how many iterations, during which we fulfilled the
    //  epsilonconvergencethreshold_, we will declare a path to have been
    //  converged to its fixed path/attractor.

    //sparse roadmap parameters
    sparseDeltaFraction_ = 0.1; //original is 0.25 (SMLR). We used 0.15 for WAFR
    maxFailures_ = 5000;
    epsilonPathEquivalence_ = 0.5;
    epsilonConvergenceThreshold_ = 1e-2;
    NsubtresholdIterations_ = 100;

    // Working for all, except the Kleinbottle
    // sparseDeltaFraction_ = 0.1;
    // maxFailures_ = 5000;
    // epsilonPathEquivalence_ = 0.3;
    // epsilonConvergenceThreshold_ = 1e-2;
    // NsubtresholdIterations_ = 10;

    if (hasBaseSpace())
    {
        static_cast<BundleSpaceGraph *>(getChild())->getGraphSampler()->disablePathBias();
    }
}

PathSpaceSparse::~PathSpaceSparse()
{
}

bool PathSpaceSparse::hasConverged()
{
    return BaseT::hasConverged() && allPathsHaveConverged();
}

void PathSpaceSparse::setup()
{
    BaseT::setup();
}

void PathSpaceSparse::grow()
{
    BaseT::grow();

    //from time to time add some more goal configurations to our roadmap
    if(pis_.getSampledGoalsCount() < getGoalPtr()->maxSampleCount())
    {
        const base::State *state = pis_.nextGoal();
        Configuration *qgoal = new Configuration(getBundle(), state);
        qgoal->isGoal = true;
        addConfiguration(qgoal);
        goalConfigurations_.push_back(qgoal);
    }

    //update paths in minima map by doing additional optimization steps
    //NOTE: this part can be skipped for optimizers which are idempotent (i.e.
    //Let o be the optimizer and p the path, then o is idempotent if o(p)=o(o(p)))

    std::lock_guard<std::recursive_mutex> 
      guard(getLocalMinimaTree()->getLock());

    if(getNumberOfPaths()<=0)
    {
      return;
    }

    if(allPathsHaveConverged()) return;

    unsigned int index = getRandomNonConvergedPathIndex();

    ompl::base::PathPtr& path = getPathPtrNonConst(index);
    geometric::PathGeometric &gpath = 
      static_cast<geometric::PathGeometric &>(*path);

    optimizePath(gpath);

    double cost = getPathCost(gpath);

    updatePath(index, path, cost);

    for (unsigned int i = 0; i < getNumberOfPaths(); i++)
    {
        if(i == index) continue;
        // if(!isPathConverged(i)) continue;
        bool equal = arePathsEquivalent(path, getPathPtr(i));
        if(equal)
        {
            removePath(index);
            break;
        }
    }
}

unsigned int PathSpaceSparse::getRandomNonConvergedPathIndex()
{
    std::vector<unsigned int> pIndices;
    for(unsigned int k = 0; k < getNumberOfPaths(); k++)
    {
       if(isPathConverged(k)) continue;
       pIndices.push_back(k);
    }

    if(pIndices.size() <= 0)
    {
      throw "AllPathsConverged";
    }

    int s = rng_.uniformInt(0, pIndices.size()-1);
    unsigned int index = pIndices.at(s);
    return index;
}

double PathSpaceSparse::getPathCost(const geometric::PathGeometric& path)
{
    ompl::base::OptimizationObjectivePtr obj = getOptimizationObjectivePtr();
    double cost = path.cost(obj).value();
    // double length = path.length();
    return cost;
}

ompl::base::PathPtr& PathSpaceSparse::getSolutionPathByReference()
{
    return getBestPathPtrNonConst();
}

const BundleSpaceGraph::Vertex 
PathSpaceSparse::getNearestStartVertex(
const std::pair<BundleSpaceGraph::Edge, bool> &edge)
{
    if(startConfigurations_.size() > 0)
    {
        std::cout << "NYI" << std::endl;
        exit(0);
    }else
    {
        return getStartIndex();
    }
}

const BundleSpaceGraph::Vertex 
PathSpaceSparse::getNearestGoalVertex(
const std::pair<BundleSpaceGraph::Edge, bool> &edge)
{
    const Vertex v = boost::source(edge.first, getGraph());
    if(goalConfigurations_.size() > 0)
    {
        double dBest = dInf;
        Vertex vBest = 0;
        bool foundBest = false;
        foreach (Configuration *goal, goalConfigurations_)
        {
            bool eligible = sameComponent(v, goal->index);
            if(eligible)
            {
                double d = distance(getGraph()[v], goal);
                if(d < dBest)
                {
                    dBest = d;
                    vBest = goal->index;
                    foundBest = true;
                }
            }
        }
        if(foundBest) return vBest;
    }
    return getGoalIndex();
}

const std::pair<BundleSpaceGraph::Edge, bool> 
PathSpaceSparse::addEdge(const Vertex a, const Vertex b)
{
    const std::pair<Edge, bool> edge = BaseT::addEdge(a, b);

    const Vertex vStart = getNearestStartVertex(edge);
    const Vertex vGoal = getNearestGoalVertex(edge);
    const Vertex v = boost::source(edge.first, getGraph());

    checkPath(v, vStart, vGoal);

    return edge;
}

ompl::base::PathPtr PathSpaceSparse::constructPath(const Vertex v, const Vertex vStart, const Vertex vGoal)
{
    base::PathPtr path1 = getPath(vStart, v);
    base::PathPtr path2 = getPath(v, vGoal);

    if(!path1)
    {
        return path2;
    }
    if(!path2)
    {
        return path1;
    }

    geometric::PathGeometric &gpath1 = 
      static_cast<geometric::PathGeometric &>(*path1);
    geometric::PathGeometric &gpath2 = 
      static_cast<geometric::PathGeometric &>(*path2);
    std::vector<base::State*> states = gpath2.getStates();
    for(uint k = 0; k < states.size(); k++)
    {
      base::State* sk = states.at(k);
      gpath1.append(sk);
    }
    return path1;
}

void PathSpaceSparse::checkPath(const Vertex v, const Vertex vStart, const Vertex vGoal)
{
  //how to best get additional (diverse) goal states?
    bool eligible = sameComponent(vStart, v) && sameComponent(v, vGoal);

    // std::cout << "Goal states:" << goalConfigurations_.size() << std::endl;
    // std::cout << "Vertices: v=" << v << " vStart=" 
    //   << vStart << " vGoal=" << vGoal << std::endl;

    if (eligible)
    {
        ompl::base::PathPtr path = constructPath(v, vStart, vGoal);
        geometric::PathGeometric &gpath = 
          static_cast<geometric::PathGeometric &>(*path);

        //TODO: encapsulate the functionality into "AddPathToDatabase", then use
        //this for database sparsification.

        // gpath.interpolate();

        optimizePath(gpath);
        double pathcost = getPathCost(gpath);

        bool isVisible = false;


        OMPL_INFORM("** Testing path with cost %f", pathcost);
        for (uint i = 0; i < getNumberOfPaths(); i++)
        {
            //NOTE: 
            //computing maximum metric over finely discretized paths might be 
            //a bit costly, but definitely more efficient than path visibility
            //(which would not only compute distance, but also edge feasibility between
            //path segments). 

            isVisible = arePathsEquivalent(path, getPathPtr(i));

            if (isVisible)
            {
                if (pathcost < PathSpace::getPathCost(i))
                {
                    OMPL_INFORM("Update minima %d with path.", i);
                    updatePath(i, path, pathcost);
                }
                //DEBUG
                // isVisible = false;
                break;
            }
        }

        if (!isVisible)
        {
            addPath(path, pathcost);
        }

        if (pathcost < bestCost_)
        {
            bestCost_ = pathcost;
        }

        // std::cout << "Best cost found: " << bestCost_ << std::endl;
        // OMPL_INFORM("Found %d path classes.", getNumberOfPaths());
    }
}

bool PathSpaceSparse::arePathsEquivalent(
    ompl::base::PathPtr path1,
    ompl::base::PathPtr path2)
{
    geometric::PathGeometric &gpath1 = 
      static_cast<geometric::PathGeometric &>(*path1);
    geometric::PathGeometric &gpath2 = 
      static_cast<geometric::PathGeometric &>(*path2);

    //Early termination if length is too large
    // double d1 = gpath1.length();
    // double d2 = gpath2.length();
    // double dabs = fabs(d1 - d2);
    // if(dabs > 1e-1) return false;

    double d = pathMetric_MaxMin(
        gpath1.getStates(), gpath2.getStates(), getBundle());
    return (d < epsilonPathEquivalence_); 
}

void PathSpaceSparse::optimizePath(geometric::PathGeometric& gpath)
{ 
    //NOTES
    // * You can insert your own optimizer at this point. 
    // * The algorithm should be agnostic to the implementation, but we assume
    // that it asymptotically converges, always lowers the cost and is
    // deterministic.

    geometric::PathGeometric pathOld(gpath);
    double costOld = getPathCost(pathOld);

    bool valid = true;

    if (getBundle()->getStateSpace()->getType() == base::STATE_SPACE_SO2)
    {
        // optimizer_->collapseCloseVertices(gpath);
    }else{
        gpath.subdivide();
        optimizer_->perturbPath(gpath, 0.1, 1000, 1000);
        // optimizer_->smoothBSpline(gpath);
        // optimizer_->perturbPath(gpath, 0.1);
        valid = optimizer_->simplifyMax(gpath);

        // optimizer_->reduceVertices(gpath);
        // optimizer_->collapseCloseVertices(gpath);
    }
    gpath.interpolate();

    double costNew = getPathCost(gpath);
    // if(costNew > (costOld))
    // {
    //     valid = false;
    // }

    if(!valid)
    {
        gpath = pathOld;
        OMPL_ERROR("RESTORED PATH (cost %.2f -> %.2f -> %.2f)", costOld, costNew, getPathCost(gpath));
    }

    return;

    base::ProblemDefinitionPtr pdef = getProblemDefinition();
    base::OptimizationObjectivePtr obj = getOptimizationObjectivePtr();
    // const double rangeRatio = 0.01;

    if (gpath.getStateCount() < 3)
        return;

    // unsigned int maxSteps = gpath.getStateCount();

    unsigned int maxEmptySteps = 1000;//floor(0.5*gpath.getStateCount());

    // const base::SpaceInformationPtr &si = gpath.getSpaceInformation();
    std::vector<base::State *> &states = gpath.getStates();
	
    unsigned int rmStates = 0;
		for (unsigned int nochange = 0; nochange < maxEmptySteps; nochange++)
		{
				int count = states.size();
        if(count < 3) break;
				int maxN = count - 1;

        {
          int p1 = rng_.uniformInt(0, maxN);
          int p2 = std::min(maxN, p1 + 2);
          int p3 = std::min(maxN, p1 + 1);

          base::Cost d12 = obj->motionCost(states[p1], states[p2]);
          base::Cost d13 = obj->motionCost(states[p1], states[p3]);
          base::Cost d32 = obj->motionCost(states[p3], states[p2]);

          base::Cost d132 = obj->combineCosts(d13, d32);

          if(obj->isCostBetterThan(d12, d132))
          {
              states.erase(states.begin() + p1 + 1, states.begin() + p2);
              nochange = 0;
              rmStates++;
              continue;
          }
        }
		}
}

