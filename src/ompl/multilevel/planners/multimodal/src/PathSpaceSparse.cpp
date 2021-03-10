#include <ompl/multilevel/planners/multimodal/PathSpaceSparse.h>
#include <ompl/multilevel/planners/multimodal/datastructures/PathSpaceMetrics.h>
#include <ompl/tools/config/SelfConfig.h>
#include <ompl/datastructures/NearestNeighbors.h>
#include <ompl/datastructures/PDF.h>
#include <boost/math/constants/constants.hpp>
#include <ompl/multilevel/datastructures/graphsampler/GraphSampler.h>
#include <ompl/multilevel/datastructures/pathrestriction/PathRestriction.h>
#include <ompl/multilevel/datastructures/PlannerDataVertexAnnotated.h>
#include <ompl/multilevel/planners/multimodal/datastructures/PathVisibilityChecker.h>

#include <ompl/util/Exception.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/objectives/MaximizeMinClearanceObjective.h>

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
    pathVisibilityChecker_ = new PathVisibilityChecker(getBundle());

    setName("PathSpaceSparse" + std::to_string(id_));

    sparseDeltaFraction_ = 0.1; //original is 0.25 (SMLR). We used 0.15 for WAFR
    maxFailures_ = 1000;

    if (hasBaseSpace())
    {
        static_cast<BundleSpaceGraph *>(getBaseBundleSpace())->getGraphSampler()->disablePathBias();
    }
}

PathSpaceSparse::~PathSpaceSparse()
{
    delete pathVisibilityChecker_;
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

    if(getNumberOfPaths()<=0) return;

    unsigned int index = rng_.uniformInt(0, getNumberOfPaths()-1);
    if(isPathConverged(index)) return;

    ompl::base::PathPtr& path = getPathPtrNonConst(index);
    geometric::PathGeometric &gpath = 
      static_cast<geometric::PathGeometric &>(*path);

    optimizePath(gpath);

    double cost = gpath.length(); //TODO: make it arbitrary cost

    updatePath(index, path, cost);

    for (unsigned int i = 0; i < getNumberOfPaths(); i++)
    {
        if(i == index) continue;
        if(!isPathConverged(i)) continue;
        bool equal = arePathsEquivalent(path, getPathPtr(i));
        if(equal)
        {
            removePath(index);
            break;
        }
    }
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

        optimizePath(gpath);

        // std::cout << " OPTIMIZED to path with cost " << gpath.length() 
        //   << " (" << states.size() << " states)." << std::endl;

        bool isVisible = false;

        double pathcost = gpath.length();

        OMPL_INFORM("** Testing path with length %f",gpath.length());
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
                if (pathcost < getPathCost(i))
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
    // std::cout << "Metric cost difference: " << d << std::endl;
    geometric::PathGeometric &gpath1 = 
      static_cast<geometric::PathGeometric &>(*path1);
    geometric::PathGeometric &gpath2 = 
      static_cast<geometric::PathGeometric &>(*path2);
    double d1 = gpath1.length();
    double d2 = gpath2.length();
    double dabs = fabs(d1 - d2);

    //Epsilon Equivalent cost 
    if(dabs > 1e-1) return false;

    double d = pathMetric_MaxMin(
        gpath1.getStates(), gpath2.getStates(), getBundle());
    return (d < 0.3); //TODO: Make this a bit more rigorous (how about relating it to epsilon?)
}
void PathSpaceSparse::optimizePath(geometric::PathGeometric& gpath)
{ 
    if (getBundle()->getStateSpace()->getType() == base::STATE_SPACE_SO2)
    {
        // std::cout << "SO2 detected. Optimizer disabled." << std::endl;
        // optimizer_->collapseCloseVertices(gpath);
    }else{
        // optimizer_->perturbPath(gpath, 0.1, 1000, 1000);
        // optimizer_->smoothBSpline(gpath);
        optimizer_->simplifyMax(gpath);
        // optimizer_->reduceVertices(gpath);
        // optimizer_->collapseCloseVertices(gpath);
    }

    gpath.interpolate();
    return;

    base::ProblemDefinitionPtr pdef = getProblemDefinition();
    base::OptimizationObjectivePtr obj = getOptimizationObjectivePtr();
    // const double rangeRatio = 0.01;

    // double dmax = gpath.length();

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

