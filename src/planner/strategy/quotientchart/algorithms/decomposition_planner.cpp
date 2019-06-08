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
  s1_ = s1;
  s2_ = s2;
  s_interp1 = si_->allocState();
  s_interp2 = si_->allocState();
  //std::vector<double> d1_;
  //std::vector<double> d2_;
  //double length1_ = 0;
  //double length2_ = 0;
  computePathLength(s1_, d1_, length1_);
//	std::cout << "distances vector size 1" << d1_.size() << std::endl;
//	std::cout << "path vector size 1" << s1.size() << std::endl;
  computePathLength(s2_, d2_, length2_);
//	std::cout << "distances vector size 2" << d2_.size() << std::endl;
//        std::cout << "path vector size 2" << s2.size() << std::endl;

}

virtual bool isValid(const ob::State *state) const
{
const ob::RealVectorStateSpace::StateType *RnSpace = state->as<ob::RealVectorStateSpace::StateType>();
      double t1 = RnSpace->values[0] * length1_;
      double t2 = RnSpace->values[1] * length2_;
//	std::cout << "value at 0 is" << RnSpace->values[0] << std::endl;
//	std::cout << "value at 1 is" << RnSpace->values[1] << std::endl;

//	uint index1 = (int) t1;
//	uint index2 = (int) t2;
//std::cout << t1  << "should be smaller than " << length1_ << std::endl;
//std::cout << t2 << "should be smaller than" << length2_ << std::endl;

	createStateAt(s1_, length1_, d1_, t1, s_interp1);
	createStateAt(s2_, length2_, d2_, t2, s_interp2);
//std::cout << "arriving here" << std::endl;
//std::cout << "checkMotion" << si_->checkMotion(s_interp1, s_interp2) << std::endl;
return si_->checkMotion(s_interp1, s_interp2);
//return false;
}

//computes the distance between each two adjacent states in the path and the overall path-length
//saves the accumulated (!) distance (distance from start to the point at this index) for each point in the stateDistances vector
//computes the overall pathLength which ist equal to the last entry of stateDistances
void computePathLength(const std::vector<ob::State*> &path, std::vector<double> &stateDistances, double &pathLength) {
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
void createStateAt(const std::vector<ob::State*> &path, const double &pathLength, const std::vector<double> &distances, const double newPosition, ob::State* s_interpolate) const {
	if(distances.size() > 1) {
		uint idx = 0;
		bool minIndex = false;
		//search for the correct index in our given path
		//if just to make the search for the correct index idx faster
		//if(newPosition < pathLength/2) {
			for(uint i = 0; i < distances.size(); i++) {
				if (distances.at(i) > newPosition) {
					if (i > 0) {
						idx = i-1;
						break;
					} else {
						//idx = 0
						minIndex = true;
						break;
					}
				}
			}
		//} else {
		//for(uint i = distances.size() - 2; i > 0; i--) {
                //                if (distances.at(i) < newPosition) {
                //                        idx = i;
                //                        break;
                //                }
                //        }
		//}

		////if(newPosition >= pathLength/2) {
        	////        for(uint i = distances.size() - 2; i > 0; i--) {
                ////	        if (distances.at(i) < newPosition) {
                ////        	        idx = i;
		////			break;
                ////        	}
                ////	}
		////}
		
		//line fraction gives the portion of where to interpolate between state i and i+1
	        if ((idx < distances.size() - 1) && (minIndex == false)) {
			double lineFraction = (newPosition - distances.at(idx))/(distances.at(idx+1) - distances.at(idx));
        		//std::cout << "running this" << std::endl;
			if (lineFraction != 0) {
				si_->getStateSpace()->interpolate(path.at(idx), path.at(idx+1), lineFraction, s_interpolate);
        		//return s_interpolate;
			} else {
				s_interpolate = path.at(idx);
			}
		} else if (idx == distances.size() - 1) {
			double lineFraction = (newPosition - distances.at(idx))/(pathLength - distances.at(idx));
			si_->getStateSpace()->interpolate(path.at(idx), path.at(idx+1), lineFraction, s_interpolate);
		} else if ((idx == 0) && (minIndex == true)) {
			//std::cout << "running that" << std::endl;
			minIndex = false;
			double lineFraction = newPosition/distances.at(0);
			si_->getStateSpace()->interpolate(path.at(0), path.at(1), lineFraction, s_interpolate);
		}
	} else if (distances.size() == 1) {
		//ob::State *s_interpolate = si_->allocState();
		double lineFraction = newPosition/distances.at(0);
                if (lineFraction != 0) {
                        si_->getStateSpace()->interpolate(path.at(0), path.at(1), lineFraction, s_interpolate);
                        //return s_interpolate;
        	}
	}
}


private:

ob::SpaceInformationPtr si_local_;
vector<ob::State*> s1_;
vector<ob::State*> s2_;
vector<double> d1_;
vector<double> d2_;
double length1_;
double length2_;
ob::State* s_interp1;
ob::State* s_interp2;
};


//https://de.wikipedia.org/wiki/Strategie_(Entwurfsmuster)
bool DecompositionPlanner::IsPathVisible(std::vector<ob::State*> &s1, std::vector<ob::State*> &s2)
{

//    virtual bool isValid(const ob::State* state) const{
//
//      const ob::RealVectorStateSpace::StateType *RnSpace = state->as<ob::RealVectorStateSpace::StateType>();
//      double t1 = RnSpace->values[0];
//      double t2 = RnSpace->values[1];
//
//      Config q1 = path1->Eval(t1);
//      Config q2 = path2->Eval(t2);
//
//      ob::ScopedState<> s1 = cspace->ConfigToOMPLState(q1);
//      ob::ScopedState<> s2 = cspace->ConfigToOMPLState(q2);
//
//      bool isfeasible = si_path->checkMotion(s1.get(),s2.get());
//
      //cspace->SpacePtr()->freeState(s1);
      //cspace->SpacePtr()->freeState(s2);
//
//      return isfeasible;
//    }
//
//  private:
//
//    CSpaceOMPL *cspace;
//    ob::SpaceInformationPtr si_path;
//    PathPiecewiseLinear *path1;
//    PathPiecewiseLinear *path2;
//
//};

	float max__planning_time_path_path = 0.5;
	float epsilon_goalregion = 0.01;
//si->setup();

//get start and goal state
//ob::State *s_start = G[v_start]->state;
//ob::State *s_goal = G[v_goal]->state;


//isVisiblePathPath
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
//ss.setStateValidityChecker(isValid);
ob::PlannerPtr ompl_planner = std::make_shared<og::RRT>(si_local);
ss.setStartAndGoalStates(start, goal, epsilon_goalregion);
ss.setPlanner(ompl_planner);
ss.setup();

////set objective to infinite path to just return first solution
ob::ProblemDefinitionPtr pdef = ss.getProblemDefinition();
//pdef->setOptimizationObjective( getThresholdPathLength(si_local) );

ob::PlannerTerminationCondition ptc( ob::timedPlannerTerminationCondition(max__planning_time_path_path) );

ss.solve(ptc);
bool solved = ss.haveExactSolutionPath();

//ompl::msg::setLogLevel(ompl::msg::LOG_INFO);
if (solved) {
//std::cout << "RRT solved" << std::endl;
}
return solved;

}

//############################################################################
//Some Useful Function for PathVisibility Computation:
//############################################################################
//typedef std::vector<Vertex> Path;
////(0) How to get start and goal states
//ob::State *s_start = G[v_start]->state;
//ob::State *s_goal = G[v_goal]->state;

////(1) How to get an ompl::base::State* from the vertex v
//ob::State *sv = G[v]->state;

////(2) How to interpolate between two states
//ob::State *s_interpolated = si_->allocState();
//si_->getStateSpace()->interpolate(s_start, s_goal, 0.5, s_interpolated);

////(2) How to Iterate through subgraph G
//foreach( const Vertex vg, boost::vertices(G))
//{
//  ob::State *sg = G[vg]->state;
//  si_->getStateSpace()->interpolate(sv, sg, 0.2, s_interpolated);
//}

////(3) How to get shortest path
////  v_start and v_goal are start and goal vertex (from quotientchartsubgraph)
//Path p_start_goal = GetPathOnGraph(v_start, v_goal);
//Path p_v = GetPathOnGraph(v_start, v, v_goal);

////(4) How to check if linear path between two states is feasible
//if(si_->checkMotion(s_start,s_goal))
//{
//  std::cout << "Feasible Path exists between states" << G[v_start] << " and " << G[v_goal] << std::endl;
//}
//############################################################################
  ////(4) How to check if linear path between two states is feasible
  //if(si_->checkMotion(s_start,s_goal))
  //{
  //  std::cout << "Feasible Path exists between states" << G[v_start] << " and " << G[v_goal] << std::endl;
  //}
  //virtual bool ompl::base::MotionValidator::checkMotion 	( 	const State *  	s1,
  //		const State *  	s2 
  //	) 		const
  //############################################################################

