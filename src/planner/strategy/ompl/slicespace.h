#pragma once

#include <ompl/geometric/planners/PlannerIncludes.h>
#include <ompl/datastructures/NearestNeighbors.h>
#include <ompl/base/Cost.h>
#include <ompl/base/goals/GoalRegion.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/pending/disjoint_sets.hpp>

namespace ob = ompl::base;
namespace og = ompl::geometric;
//abstract away a boost graph plus some convenience functions of PRM

namespace ompl
{
    namespace magic
    {
        static const unsigned int MAX_RANDOM_BOUNCE_STEPS = 5;
        static const double ROADMAP_BUILD_TIME = 0.01;
        static const unsigned int DEFAULT_NEAREST_NEIGHBORS = 10;
    }
}
class GoalRegionEdge: public ob::GoalRegion{
  public:
  GoalRegionEdge(const ob::SpaceInformationPtr &si):
    GoalRegion(si) {}

  virtual double distanceGoal(const ob::State *qompl) const override{
    const ob::RealVectorStateSpace::StateType *qomplRnSpace = qompl->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0);
    //const ob::SO3StateSpace::StateType *qomplSO3 = qompl->as<ob::CompoundState>()->as<ob::SO3StateSpace::StateType>(1);
    double d = fabs(1.0 - qomplRnSpace->values[0]);
    return d;
  }
};
namespace ompl
{
  namespace base
  {
      OMPL_CLASS_FORWARD(OptimizationObjective);
  }
  namespace geometric
  {
    class SliceSpace: public ob::Planner{

        struct vertex_state_t
        {
            typedef boost::vertex_property_tag kind;
        };

        struct vertex_total_connection_attempts_t
        {
            typedef boost::vertex_property_tag kind;
        };

        struct vertex_successful_connection_attempts_t
        {
            typedef boost::vertex_property_tag kind;
        };
        typedef boost::adjacency_list<
            boost::vecS, boost::vecS, boost::undirectedS,
            boost::property<
                vertex_state_t, ob::State *,
                boost::property<
                    vertex_total_connection_attempts_t, unsigned long int,
                    boost::property<vertex_successful_connection_attempts_t, unsigned long int,
                                    boost::property<boost::vertex_predecessor_t, unsigned long int,
                                                    boost::property<boost::vertex_rank_t, unsigned long int>>>>>,
            boost::property<boost::edge_weight_t, ob::Cost>>
            Graph;
        typedef boost::graph_traits<Graph>::vertex_descriptor Vertex;
        typedef boost::graph_traits<Graph>::edge_descriptor Edge;
        typedef std::shared_ptr<NearestNeighbors<Vertex>> RoadmapNeighbors;
        typedef std::function<const std::vector<Vertex> &(const Vertex)> ConnectionStrategy;
        typedef std::function<bool(const Vertex &, const Vertex &)> ConnectionFilter;

      public:
        SliceSpace(const ob::SpaceInformationPtr &si);

        ~SliceSpace() override{
          si->freeStates(xstates);
        }

        base::PathPtr GetSolutionPath(){
          ob::PathPtr sol;
          checkForSolution(sol);
          return sol;
        }
        bool hasSolution(){
          return addedNewSolution_;
        }

        void Grow(double t = magic::ROADMAP_BUILD_TIME*3);
        double volume;

        template <template <typename T> class NN>
        void setNearestNeighbors()
        {
            if (nn_ && nn_->size() == 0)
                OMPL_WARN("Calling setNearestNeighbors will clear all states.");
            clear();
            nn_ = std::make_shared<NN<Vertex>>();
            connectionStrategy_ = ConnectionStrategy();
            if (isSetup())
                setup();
        }

        void setProblemDefinition(const base::ProblemDefinitionPtr &pdef) override;

        void setConnectionFilter(const ConnectionFilter &connectionFilter)
        {
            connectionFilter_ = connectionFilter;
        }

        void getPlannerData(base::PlannerData &data) const override;
        base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;

        void clearQuery();
        void clear() override;
        void setup() override;
    protected:

        base::Cost costHeuristic(Vertex u, Vertex v) const;
        double distanceFunction(const Vertex a, const Vertex b) const;
        ob::SpaceInformationPtr si;
        std::vector<ob::State *> xstates;

        base::ValidStateSamplerPtr sampler_;
        base::StateSamplerPtr simpleSampler_;
        RoadmapNeighbors nn_;
        Graph graph;

        boost::property_map<Graph, vertex_state_t>::type stateProperty_;
        boost::property_map<Graph, vertex_total_connection_attempts_t>::type totalConnectionAttemptsProperty_;
        boost::property_map<Graph, vertex_successful_connection_attempts_t>::type
            successfulConnectionAttemptsProperty_;
        boost::property_map<Graph, boost::edge_weight_t>::type weightProperty_;
        boost::disjoint_sets<boost::property_map<Graph, boost::vertex_rank_t>::type,
                             boost::property_map<Graph, boost::vertex_predecessor_t>::type> disjointSets_;
        ConnectionStrategy connectionStrategy_;
        ConnectionFilter connectionFilter_;
        RNG rng_;
        bool addedNewSolution_{false};
        base::OptimizationObjectivePtr opt_;
        unsigned long int iterations_{0};
        base::Cost bestCost_{std::numeric_limits<double>::quiet_NaN()};
        std::vector<Vertex> startM_;
        std::vector<Vertex> goalM_;

        Vertex addMilestone(base::State *state);

        void uniteComponents(Vertex m1, Vertex m2);
        bool sameComponent(Vertex m1, Vertex m2);
        void growRoadmap(const base::PlannerTerminationCondition &ptc, base::State *workState);
        void expandRoadmap(const base::PlannerTerminationCondition &ptc, std::vector<base::State *> &workStates);

        void checkForSolution(base::PathPtr &solution);
        bool maybeConstructSolution(const std::vector<Vertex> &starts, const std::vector<Vertex> &goals,
                                    base::PathPtr &solution);
        ompl::base::PathPtr constructSolution(const Vertex &start, const Vertex &goal);
    };

  };
};
