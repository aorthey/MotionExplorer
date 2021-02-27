#include <ompl/multilevel/planners/multimodal/PathSpaceSparse.h>
#include <ompl/multilevel/planners/multimodal/datastructures/PathSpaceMetrics.h>
#include <ompl/tools/config/SelfConfig.h>
#include <ompl/datastructures/NearestNeighbors.h>
#include <ompl/datastructures/PDF.h>
#include <boost/foreach.hpp>
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

#define foreach BOOST_FOREACH

using namespace ompl::multilevel;

PathSpaceSparse::PathSpaceSparse(const base::SpaceInformationPtr &si, BundleSpace *parent_)
  : PathSpace(this), BaseT(si, parent_)
{
    pathVisibilityChecker_ = new PathVisibilityChecker(getBundle());

    setName("PathSpaceSparse" + std::to_string(id_));

    sparseDeltaFraction_ = 0.1; //original is 0.25
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

void PathSpaceSparse::setup()
{
    BaseT::setup();
}

ompl::base::PathPtr& PathSpaceSparse::getSolutionPathByReference()
{
  return getBestPathPtrNonConst();
}

const std::pair<BundleSpaceGraph::Edge, bool> 
PathSpaceSparse::addEdge(const Vertex a, const Vertex b)
{
    auto edge = BaseT::addEdge(a, b);

    const Vertex vt = boost::target(edge.first, getGraph());
    const Vertex vs = boost::source(edge.first, getGraph());
    const Vertex vStart = getStartIndex();
    const Vertex vGoal = getGoalIndex();
    if(vt!=vStart && vt!=vGoal)
    { 
      checkPath(vt, vStart, vGoal);
    }else{
      if(vs!=vStart && vs!=vGoal)
      {
        checkPath(vs, vStart, vGoal);
      }
      //do not checkPath if only one edge exists between start/goal
    }

    return edge;
}

void PathSpaceSparse::checkPath(const Vertex v, const Vertex vStart, const Vertex vGoal)
{
    bool eligible = sameComponent(vStart, v) && sameComponent(v, vGoal);

    if (eligible)
    {
        ompl::base::PathPtr pPath = getPath(vStart, v);
        ompl::base::PathPtr pPath2 = getPath(v, vGoal);

        geometric::PathGeometric &gpath = 
          static_cast<geometric::PathGeometric &>(*pPath);
        geometric::PathGeometric &gpath2 = 
          static_cast<geometric::PathGeometric &>(*pPath2);

        std::vector<base::State*> states = gpath2.getStates();
        for(uint k = 0; k < states.size(); k++)
        {
          base::State* sk = states.at(k);
          gpath.append(sk);
        }

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
            double d = pathMetric_MaxMin(
                gpath.getStates(), getPathStates(i), getBundle());
            // std::cout << "Metric cost difference: " << d << std::endl;
            OMPL_INFORM("Distance to minima %d is %f", i, d);
            isVisible = (d < 0.3);
            if (isVisible)
            {
                if (pathcost < getPathCost(i))
                {
                    OMPL_INFORM("Update minima %d with path.", i);
                    updatePath(i, pPath, pathcost);
                }
                //DEBUG
                // isVisible = false;
                break;
            }
        }

        if (!isVisible)
        {
            addPath(pPath, pathcost);
        }

        if (pathcost < bestCost_)
        {
            bestCost_ = pathcost;
        }

        // std::cout << "Best cost found: " << bestCost_ << std::endl;
        // OMPL_INFORM("Found %d path classes.", getNumberOfPaths());
    }
}

//void PathSpaceSparse::grow()
//{
//    if (firstRun_)
//    {
//        init();

//        firstRun_ = false;

//        vGoal_ = addConfiguration(qGoal_);

//        // if (hasBaseSpace())
//        // {
//        //     if (getPathRestriction()->hasFeasibleSection(qStart_, qGoal_))
//        //     {
//        //         if (sameComponent(vStart_, vGoal_))
//        //         {
//        //             hasSolution_ = true;
//        //         }
//        //     }
//        // }
//    }

//    //(1) Get Random Sample
//    if (!sampleBundleValid(xRandom_->state))
//        return;

//    //(2) Add Configuration if valid
//    Configuration *xNew = new Configuration(getBundle(), xRandom_->state);

//    //TODO: problem is that we do not check here if a new edge is added! (which
//    //is what we would ultimately care about). Maybe we need to highjack the
//    //addEdge function
//    addConfigurationConditional(xNew);

//    if (!hasSolution_)
//    {
//        if (sameComponent(getStartIndex(), getGoalIndex()))
//        {
//            hasSolution_ = true;
//        }
//    }

//    // writeToGraphviz("graphviz"+std::to_string(getNumberOfVertices()));
//}

void PathSpaceSparse::optimizePath(geometric::PathGeometric& gpath)
{ 
    if (getBundle()->getStateSpace()->getType() == base::STATE_SPACE_SO2)
    {
        std::cout << "SO2 detected. Optimizer disabled." << std::endl;
        // optimizer_->collapseCloseVertices(gpath);
    }else{
        // optimizer_->perturbPath(gpath, 0.1, 1000, 1000);
        optimizer_->smoothBSpline(gpath);
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

