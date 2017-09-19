#pragma once

#include "elements/path_pwl_euclid.h"

#include <ompl/base/Cost.h>
#include <ompl/base/PlannerData.h>
#include <ompl/base/PlannerDataGraph.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/StateSpace.h>
#include <list>
#include <utility>
#include <map>
#include <lemon/list_graph.h>
#include <lemon/dijkstra.h>

#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>

#include <Library/KrisLibrary/math/vector.h>
#include <Library/KrisLibrary/math3d/primitives.h>
#include <KrisLibrary/robotics/RobotKinematics3D.h> //Config

namespace ob = ompl::base;
namespace og = ompl::geometric;

using Cost = ob::Cost;
using Graph = ob::PlannerData::Graph;
using Vertex = Graph::Vertex;
using Edge = Graph::Edge;
using VIterator = Graph::VIterator;
using EIterator = Graph::EIterator;

static ob::OptimizationObjectivePtr getThresholdPathLength(const ob::SpaceInformationPtr& si)
{
  ob::OptimizationObjectivePtr obj(new ob::PathLengthOptimizationObjective(si));
  obj->setCostThreshold(ob::Cost(dInf));
  return obj;
}
Vector3 vertexIndexToVector(const ob::PlannerData& pd, const Vertex &v);
std::vector<Vector3> vertexIndicesToVector(const ob::PlannerData& pd, const std::vector<Vertex> &v);

//Let p,q be two paths [0,1]->M
//Linearsegmentvaliditychecker: 
//  for given s,t \in [0,1] checks if the linear segments between p(s) and q(t)
//  is valid. This is done by calling ompl's checkmotion

class LinearSegmentValidityChecker : public ob::StateValidityChecker
{
  public:
    LinearSegmentValidityChecker(const ob::SpaceInformationPtr &si, const ob::SpaceInformationPtr &si_path_, const ob::PlannerData& pd, const std::vector<Vertex> &p1, const std::vector<Vertex> &p2):
      ob::StateValidityChecker(si), si_path(si_path_)
    {
      std::vector<Vector3> s1;
      std::vector<Vector3> s2;

      for(uint k = 0; k < p1.size(); k++){
        Vector3 v = vertexIndexToVector(pd, p1.at(k));
        s1.push_back(v);
      }
      for(uint k = 0; k < p2.size(); k++){
        Vector3 v = vertexIndexToVector(pd, p2.at(k));
        s2.push_back(v);
      }
      /////DEBUG

      path1 = PathPiecewiseLinearEuclidean::from_keyframes(s1);
      path1->Normalize();
      path2 = PathPiecewiseLinearEuclidean::from_keyframes(s2);
      path2->Normalize();
    }

    virtual bool isValid(const ob::State* state) const{

      const ob::RealVectorStateSpace::StateType *RnSpace = state->as<ob::RealVectorStateSpace::StateType>();
      double t1 = RnSpace->values[0];
      double t2 = RnSpace->values[1];

      Vector3 vv = path1->EvalVec3(t1);
      Vector3 ww = path2->EvalVec3(t2);

      const ob::StateSpacePtr space_path = si_path->getStateSpace();

      ob::State *q1 = space_path->allocState();
      ob::State *q2 = space_path->allocState();

      for(uint k = 0; k < 3; k++){
        q1->as<ob::RealVectorStateSpace::StateType>()->values[k] = vv[k];
        q2->as<ob::RealVectorStateSpace::StateType>()->values[k] = ww[k];
      }
    
      bool isfeasible = si_path->checkMotion(q1,q2);

      space_path->freeState(q1);
      space_path->freeState(q2);
      return isfeasible;

    }

  private:

    ob::SpaceInformationPtr si_path;
    PathPiecewiseLinearEuclidean *path1;
    PathPiecewiseLinearEuclidean *path2;

};

//bool testVisibilityRRT(const ob::PlannerData& pd, const ob::SpaceInformationPtr &si_path_space, const std::vector<Vertex> &p1, const std::vector<Vertex> &p2);
//std::vector< std::vector< Vector3 >> ComputeShortestPathsLemon(ob::PlannerData& pd_in, const ob::OptimizationObjective& opt);

class OnetopicPathSpaceModifier{
  public:
    explicit OnetopicPathSpaceModifier( ob::PlannerData& pd_in, const ob::OptimizationObjective& opt );

    std::vector< std::vector< Vector3 >> GetVectorPaths();
    std::vector< std::vector< Config >> GetConfigPaths();


  private:
    bool testVisibilityRRT(const ob::PlannerData& pd, const ob::SpaceInformationPtr &si_path_space, const std::vector<Vertex> &p1, const std::vector<Vertex> &p2);
    std::vector< std::vector< Vector3 >> ComputeShortestPathsLemon(ob::PlannerData& pd_in, const ob::OptimizationObjective& opt);

    std::vector< std::vector< Vector3 >> vector_paths;
    std::vector< std::vector< Config >> config_paths;


};
