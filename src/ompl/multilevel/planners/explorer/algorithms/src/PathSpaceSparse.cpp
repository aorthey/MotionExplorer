#include <ompl/multilevel/planners/explorer/algorithms/PathSpaceSparse.h>
#include <ompl/multilevel/planners/explorer/datastructures/PathSpaceMetrics.h>
#include <ompl/tools/config/SelfConfig.h>
#include <ompl/datastructures/NearestNeighbors.h>
#include <ompl/datastructures/PDF.h>
#include <boost/foreach.hpp>
#include <boost/math/constants/constants.hpp>
#include <ompl/multilevel/datastructures/graphsampler/GraphSampler.h>
#include <ompl/multilevel/datastructures/pathrestriction/PathRestriction.h>
#include <ompl/multilevel/datastructures/PlannerDataVertexAnnotated.h>
#include <ompl/multilevel/planners/explorer/datastructures/PathVisibilityChecker.h>

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


    optimizationObjective_ = std::make_shared<ompl::base::MultiOptimizationObjective>(getBundle());

    ompl::base::OptimizationObjectivePtr lengthObj =
        std::make_shared<ompl::base::PathLengthOptimizationObjective>(getBundle());
    std::static_pointer_cast<base::MultiOptimizationObjective>(optimizationObjective_)
      ->addObjective(lengthObj, 1.0);

    optimizer_ = std::make_shared<ompl::geometric::PathSimplifier>(getBundle(), base::GoalPtr(),
        optimizationObjective_);
    optimizer_->freeStates(false);

    sparseDeltaFraction_ = 0.1; //original is 0.25

    if (hasBaseSpace())
    {
        static_cast<BundleSpaceGraph *>(getBaseBundleSpace())->getGraphSampler()->disablePathBias();
    }
}

PathSpaceSparse::~PathSpaceSparse()
{
    delete pathVisibilityChecker_;
}

ompl::base::PathPtr& PathSpaceSparse::getSolutionPathByReference()
{
  return getBestPathPtrNonConst();
}

void PathSpaceSparse::optimizePath(geometric::PathGeometric& gpath)
{ 
    // optimizer_->perturbPath(gpath, 2, 1000, 1000);
    optimizer_->smoothBSpline(gpath);
    optimizer_->simplifyMax(gpath);
    gpath.interpolate();
    return;

    base::ProblemDefinitionPtr pdef = getProblemDefinition();
    base::OptimizationObjectivePtr obj = pdef->getOptimizationObjective();
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

void PathSpaceSparse::grow()
{
    if (firstRun_)
    {
        init();

        firstRun_ = false;

        vGoal_ = addConfiguration(qGoal_);


        // if (hasBaseSpace())
        // {
        //     if (getPathRestriction()->hasFeasibleSection(qStart_, qGoal_))
        //     {
        //         if (sameComponent(vStart_, vGoal_))
        //         {
        //             hasSolution_ = true;
        //         }
        //     }
        // }
    }

    //(1) Get Random Sample
    if (!sampleBundleValid(xRandom_->state))
        return;

    //(2) Add Configuration if valid
    Configuration *xNew = new Configuration(getBundle(), xRandom_->state);

    if(!addConfigurationConditional(xNew)) return;

    Vertex v = xNew->index;

    if(v>1e10)
    {
      std::cout << "Vertex " << v << std::endl;
      throw ompl::Exception("Vertex invalid");
    }


    Vertex vStart = getStartIndex();

    Vertex vGoal = getGoalIndex();

    if (!hasSolution_)
    {
        if (sameComponent(vStart, vGoal))
        {
            hasSolution_ = true;
        }
    }

    // FALL 1: Pfad ist mit Start und Ziel verbunden
    bool b = sameComponent(vStart, v) && sameComponent(v, vGoal);

    if (b)
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

        // std::cout << "Interpolate " << gpath.getStates().size();
        // gpath.interpolate();
        // std::cout << " to " << gpath.getStates().size() << std::endl;

        std::cout << "Path with cost " << gpath.length() 
          << " (" << states.size() << " states).";

        optimizePath(gpath);

        std::cout << " OPTIMIZED to path with cost " << gpath.length() 
          << " (" << states.size() << " states)." << std::endl;

        bool isVisible = false;

        double pathcost = gpath.length();

        for (uint i = 0; i < getNumberOfPaths(); i++)
        {
            // FALL 2: verbessert bestehenden pfad
            // Knoten wurde mit 2 Knoten des Pfades verbunden
            // if (pathVisibilityChecker_->IsPathVisible(gpath.getStates(), 
            //       getPath(i)))
            // if (pathVisibilityChecker_->IsPathVisible(
            //       gpath.getStates(), 
            //       getPathStatesNonConst(i)))

            //NOTE: 
            //computing maximum metric over finely discretized paths might be 
            //a bit costly, but definitely more efficient than path visibility
            //(which would not only compute distance, but also edge feasibility between
            //path segments). 
            double d = pathMetric_Maximum(
                gpath.getStates(), getPathStates(i), getBundle());
            std::cout << "Metric cost difference: " << d << std::endl;
            isVisible = (d < 0.5);
            if (isVisible)//pathcost < getPathCost(i))
            {
                updatePath(i, pPath, pathcost);
                break;
            }
        }

        if (!isVisible)
        {
            std::cout << "Add new path" << std::endl;
            addPath(pPath, pathcost);
        }

        if (pathcost < bestCost_)
        {
            bestCost_ = pathcost;
        }

        std::cout << "Best cost found: " << bestCost_ << std::endl;

        OMPL_INFORM("Found %d path classes.", getNumberOfPaths());
    }

}
