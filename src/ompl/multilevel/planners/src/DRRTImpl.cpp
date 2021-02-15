#include <ompl/multilevel/planners/DRRTImpl.h> 
//Load Differentiable Structures
#include <ompl/base/StateValidityCheckerDifferentiable.h>
#include <ompl/base/goals/GoalStateDifferentiable.h>
#include <ompl/geometric/PathGeometric.h>

#include <ompl/multilevel/datastructures/pathrestriction/PathRestriction.h>
#include <ompl/multilevel/datastructures/graphsampler/GraphSampler.h>
#include <ompl/tools/config/SelfConfig.h>
#include <boost/foreach.hpp>
#include <Eigen/Core>


#define foreach BOOST_FOREACH
using namespace ompl::multilevel;

DRRTImpl::DRRTImpl(const base::SpaceInformationPtr &si, BundleSpace *parent_)
  : BaseT(si, parent_)
{
    setName("DRRTImpl" + std::to_string(id_));
    setImportance("exponential");
    setGraphSampler("randomvertex");
    getGraphSampler()->disableSegmentBias();

    differentiableConstraint_ =
      dynamic_cast<base::StateValidityCheckerDifferentiable*>(si->getStateValidityChecker().get());

    if(differentiableConstraint_ == nullptr)
    {
        std::cout << "Constraints are not differentiable. Reverting to RRT." << std::endl;
        OMPL_WARN("Constraints are not differentiable. Reverting to RRT.");
        throw ompl::Exception("NYI");
    }else{
        hasDifferentiableConstraints_ = true;
    }
    tmp_ = getBundle()->allocState();
}

DRRTImpl::~DRRTImpl()
{
    getBundle()->freeState(tmp_);
}

void DRRTImpl::setup()
{
    differentiableGoal_ =
      std::dynamic_pointer_cast<base::GoalStateDifferentiable>(pdef_->getGoal());

    if(differentiableGoal_ == nullptr)
    {
        std::cout << "Goal is not differentiable." << std::endl;
        OMPL_WARN("Goal is not differentiable.");
        throw ompl::Exception("NYI");
    }else{
        hasDifferentiableGoal_ = true;
    }
}

Eigen::VectorXd DRRTImpl::getGradient(const ompl::base::State *state)
{
    Eigen::VectorXd dx = -differentiableConstraint_->costGradient(state);
    Eigen::VectorXd dx_goal = 0.1*(-differentiableGoal_->costGradient(state));
    return dx + dx_goal;
}

bool DRRTImpl::leapfrog(base::State *state, int steps)
{
    double step_size = 0.05;//getBundle()->getStateValidityCheckingResolution();

    std::vector<double> dstate;
    getBundle()->getStateSpace()->copyToReals(dstate, state);

    uint N = getBundle()->getStateDimension();
    Eigen::VectorXd p(N);
    p.setZero(); //TODO: get momentum from state itself


    Eigen::VectorXd dx = getGradient(state);
    p -= step_size * 0.5 * dx;

    for(int k = 0; k < steps; k++)
    {
      //q += step_size * p  # whole step
      for(uint j = 0; j < p.size(); j++)
      {
        dstate.at(j) += step_size * p[j];
      }
      getBundle()->getStateSpace()->copyFromReals(state, dstate);
      getBundle()->getStateSpace()->enforceBounds(state);
      getBundle()->printState(state);

      dx = getGradient(state);
      std::cout << "Gradient:" << dx << std::endl;
      //p -= step_size * costGradient(q[0],q[1])  # whole step
      p -= step_size * dx;
    }
    //q += step_size * p  # whole step
    for(uint k = 0; k < p.size(); k++)
    {
      dstate.at(k) += step_size * p[k];
    }
    getBundle()->getStateSpace()->copyFromReals(state, dstate);
    getBundle()->getStateSpace()->enforceBounds(state);

    dx = getGradient(state);

    p -= step_size * 0.5 * dx;

    return true;
}

BundleSpaceGraph::Configuration* DRRTImpl::leapfrogAlongPotential(Configuration *x)
{
    Configuration *xcurrent = x;
    int ctr = 0;
    std::vector<Configuration*> path;
    path.push_back(x);
    while(ctr++ < 1000)
    {
        getBundle()->getStateSpace()->copyState(tmp_, xcurrent->state);
        bool madeProgress = leapfrog(tmp_, 10);

        if(madeProgress && getBundle()->checkMotion(xcurrent->state, tmp_))
        {
            Configuration *xnext = new Configuration(getBundle(), tmp_);
            addConfiguration(xnext);
            addBundleEdge(xcurrent, xnext);
            path.push_back(xnext);
            xcurrent = xnext;
            if(differentiableGoal_->isSatisfied(tmp_)) break;
        }else{
            if(!madeProgress)
            {
              std::cout << "Did not make progress." << std::endl;
            }else{
              std::cout << "Infeasible." << std::endl;
            }
            break;
        }
    }

    auto gpath(std::make_shared<geometric::PathGeometric>(getBundle()));
    for(uint k = 0; k < path.size(); k++)
    {
      gpath->append(path.at(k)->state);
    }
    double approxdif = 0.0;
    differentiableGoal_->isSatisfied(path.back()->state, &approxdif);
    pdef_->addSolutionPath(gpath, true, approxdif, getName());

    return xcurrent;
}

bool DRRTImpl::steer(const base::State *from, 
    const Eigen::VectorXd& gradient, base::State *to)
{
    // double stepsize = 0.1;
    std::vector<double> dfrom;
    getBundle()->getStateSpace()->copyToReals(dfrom, from);

    double d = getBundle()->getStateValidityCheckingResolution();

    // double epsilon = 2*d / gradient.norm();
    double epsilon = 2*d / gradient.norm();

    for(uint k = 0; k < gradient.size(); k++)
    {
      dfrom.at(k) -= epsilon*gradient[k];
    }

    getBundle()->getStateSpace()->copyFromReals(to, dfrom);
    getBundle()->getStateSpace()->enforceBounds(to);
    return true;
}

BundleSpaceGraph::Configuration* DRRTImpl::steerAlongGradient(Configuration *x)
{
    Configuration *xcurrent = x;
    int ctr = 0;
    std::vector<Configuration*> path;
    path.push_back(x);
    while(ctr++ < 1000)
    {
        // Eigen::VectorXd dx = differentiableConstraint_->costGradient(xcurrent->state);
        // Eigen::VectorXd dx_goal = differentiableGoal_->costGradient(xcurrent->state);
        // std::cout << "Goal  gradient : " << dx_goal << std::endl;
        // std::cout << "Cnstr gradient : " << dx << std::endl;
        Eigen::VectorXd gradient = getGradient(xcurrent->state);//dx + dx_goal;

        if(gradient.norm() < 1e-4) break;

        // std::cout << "Gradient: " << gradient << std::endl;

        bool madeProgress = steer(xcurrent->state, gradient, tmp_);

        if(madeProgress && getBundle()->checkMotion(xcurrent->state, tmp_))
        {
            Configuration *xnext = new Configuration(getBundle(), tmp_);
            addConfiguration(xnext);
            addBundleEdge(xcurrent, xnext);
            path.push_back(xnext);
            xcurrent = xnext;
            if(differentiableGoal_->isSatisfied(tmp_)) break;
        }else{
            if(!madeProgress)
            {
              std::cout << "Did not make progress." << std::endl;
            }else{
              std::cout << "Infeasible." << std::endl;
            }
            
            break;
        }
    }

    auto gpath(std::make_shared<geometric::PathGeometric>(getBundle()));
    for(uint k = 0; k < path.size(); k++)
    {
      gpath->append(path.at(k)->state);
    }
    double approxdif = 0.0;
    differentiableGoal_->isSatisfied(path.back()->state, &approxdif);
    pdef_->addSolutionPath(gpath, true, approxdif, getName());

    return xcurrent;
}

void DRRTImpl::grow()
{
    //(0) If first run, add start configuration
    if (firstRun_)
    {
        init();
        firstRun_ = false;

        // Configuration *xNext = steerAlongGradient(qStart_);
        Configuration *xNext = leapfrogAlongPotential(qStart_);
        printConfiguration(xNext);
        bool satisfied = differentiableGoal_->isSatisfied(xNext->state);
        if (satisfied)
        {
            vGoal_ = addConfiguration(qGoal_);
            addEdge(xNext->index, vGoal_);
            hasSolution_ = true;
        }else{
            std::cout << "Distance:" << differentiableGoal_->cost(xNext->state) << std::endl;
        }

        return;
    }

    //(1) Get Random Sample
    // sampleBundleGoalBias(xRandom_->state);

    ////(2) Get Nearest in Tree
    //const Configuration *xNearest = nearest(xRandom_);

    ////(3) Connect Nearest to Random (within range)
    //Configuration *xNext = extendGraphTowards_Range(xNearest, xRandom_);

    ////(4) If extension was successful, check if we reached goal
    //if (xNext && !hasSolution_)
    //{
    //    if (isDynamic())
    //    {
    //        double dist;
    //        bool satisfied = goal_->isSatisfied(xNext->state, &dist);
    //        if (dist < bestCost_.value())
    //        {
    //            bestCost_ = base::Cost(dist);
    //            // std::cout << "Nearest to goal: " << dist << std::endl;
    //        }
    //        if (satisfied)
    //        {
    //            vGoal_ = xNext->index;  // addConfiguration(qGoal_);
    //            // addEdge(xNext->index, vGoal_);
    //            hasSolution_ = true;
    //        }
    //    }
    //    else
    //    {
    //        bool satisfied = goal_->isSatisfied(xNext->state);
    //        if (satisfied)
    //        {
    //            vGoal_ = addConfiguration(qGoal_);
    //            addEdge(xNext->index, vGoal_);
    //            hasSolution_ = true;
    //        }
    //    }
    //}
}
