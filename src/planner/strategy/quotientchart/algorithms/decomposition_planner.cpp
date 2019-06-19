#include "common.h"

#include "decomposition_planner.h"
#include "elements/plannerdata_vertex_annotated.h"
#include "planner/cspace/validitychecker/validity_checker_ompl.h"
#include <limits>
#include <boost/graph/astar_search.hpp>
#include <boost/graph/incremental_components.hpp>
#include <boost/property_map/vector_property_map.hpp>
#include <boost/property_map/transform_value_property_map.hpp>
#include <boost/foreach.hpp>
#include <ompl/base/State.h>
#include <ompl/base/ScopedState.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/SimpleSetup.h>
//#include <ompl/base/MotionValidator/checkMotion.cpp>

#define foreach BOOST_FOREACH

using namespace ompl::geometric;

DecompositionPlanner::DecompositionPlanner(const base::SpaceInformationPtr &si, Quotient *parent ): BaseT(si, parent)
{
  setName("DecompositionPlanner"+std::to_string(id));
}

DecompositionPlanner::~DecompositionPlanner(void)
{
}

class pathPathValidityChecker : public ob::StateValidityChecker
{
public:
  pathPathValidityChecker(const ob::SpaceInformationPtr &si_, const ob::SpaceInformationPtr &si_local, std::vector<ob::State*> s1, std::vector<ob::State*> s2) : ob::StateValidityChecker(si_)
  {
    si_local_ = si_local;
    path1_ = s1;
    path2_ = s2;
    path1_length_ = 0;
    path2_length_ = 0;
    path1_interp_state_ = si_->allocState();
    path2_interp_state_ = si_->allocState();
    path1_distances_.clear();
    path2_distances_.clear();

    computePathLength(path1_, path1_distances_, path1_length_);
    computePathLength(path2_, path2_distances_, path2_length_);
  }

  virtual bool isValid(const ob::State *state) const
  {
    const ob::RealVectorStateSpace::StateType *RnSpace = state->as<ob::RealVectorStateSpace::StateType>();
    double t1 = RnSpace->values[0] * path1_length_;
    double t2 = RnSpace->values[1] * path2_length_;
    createStateAt(path1_, path1_length_, path1_distances_, t1, path1_interp_state_);
    createStateAt(path2_, path2_length_, path2_distances_, t2, path2_interp_state_);
    return si_->checkMotion(path1_interp_state_, path2_interp_state_);
  }

  //computes the distance between each two adjacent states in the path and the overall path-length
  //saves the accumulated (!) distance (distance from start to the point at this index) for each point in the stateDistances vector
  //computes the overall pathLength which is equal to the last entry of stateDistances
  void computePathLength(const std::vector<ob::State*> &path, std::vector<double> &stateDistances, double &pathLength) 
  {
    pathLength = 0;
    if (path.size() > 1) {
      for (uint i = 0; i < path.size() - 1; i++) {
        double distStateState = si_->distance(path.at(i), path.at(i+1));
        pathLength += distStateState;
        stateDistances.push_back(pathLength);
      }
    } else {
      stateDistances.push_back(0);
    }
  }

  //creates a new state at exactly the given position within the given path
  void createStateAt(const std::vector<ob::State*> &path, const double &pathLength, const std::vector<double> &distances, const double newPosition, ob::State* s_interpolate) const 
  {

    int idx = -1;
    for(uint i = 0; i < distances.size(); i++) {
      if (distances.at(i) >= newPosition) {
        idx = i;
        break;
      }
    }

    double distanceIdxIdxNext = distances.at(idx);
    if(idx > 0) distanceIdxIdxNext -= distances.at(idx-1);

    double lineFraction = (distances.at(idx) - newPosition)/distanceIdxIdxNext;

    si_->getStateSpace()->interpolate(path.at(idx), path.at(idx+1), lineFraction, s_interpolate);

  }


  private:

  ob::SpaceInformationPtr si_local_;
  vector<ob::State*> path1_;
  vector<ob::State*> path2_;
  vector<double> path1_distances_;
  vector<double> path2_distances_;
  double path1_length_;
  double path2_length_;
  ob::State* path1_interp_state_;
  ob::State* path2_interp_state_;
};

bool DecompositionPlanner::IsPathVisible(std::vector<ob::State*> &s1, std::vector<ob::State*> &s2)
{
  float max__planning_time_path_path = 0.1;
  float epsilon_goalregion = 0.01;

  ompl::msg::setLogLevel(ompl::msg::LOG_NONE);
  ob::RealVectorBounds bounds(2);
  bounds.setLow(0);
  bounds.setHigh(1);

  ob::StateSpacePtr space = (std::make_shared<ob::RealVectorStateSpace>(2));
  ob::RealVectorStateSpace *R2 = space->as<ob::RealVectorStateSpace>();

  R2->setBounds(bounds);

  ob::ScopedState<> start(space);
  ob::ScopedState<> goal(space);
  start[0]=start[1]=0.0;
  goal[0]=goal[1]=1.0;

  og::SimpleSetup ss(space);
  const ob::SpaceInformationPtr si_local = ss.getSpaceInformation();

  ss.setStateValidityChecker( std::make_shared<pathPathValidityChecker>(si_, si_local,  s1, s2) );
  ob::PlannerPtr ompl_planner = std::make_shared<og::RRT>(si_local);
  ss.setStartAndGoalStates(start, goal, epsilon_goalregion);
  ss.setPlanner(ompl_planner);
  ss.setup();

  ////set objective to infinite path to just return first solution
  ob::ProblemDefinitionPtr pdef = ss.getProblemDefinition();
  ob::PlannerTerminationCondition ptc( ob::timedPlannerTerminationCondition(max__planning_time_path_path) );

  ss.solve(ptc);
  bool solved = ss.haveExactSolutionPath();

  ompl::msg::setLogLevel(ompl::msg::LOG_INFO);
  return solved;

}
