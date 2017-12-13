#pragma once

#include <ompl/geometric/planners/PlannerIncludes.h>
#include <ompl/datastructures/NearestNeighbors.h>
#include <ompl/base/Cost.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/pending/disjoint_sets.hpp>
#include <stack>
#include <KrisLibrary/math/infnan.h> //dInf, fInf, IsNaN(x)
using Math::dInf;

namespace ob = ompl::base;
namespace og = ompl::geometric;

//copied from ompl/geometric/planners/PRM.h plus some modifications/simplifications
namespace ompl
{
  namespace base
  {
      OMPL_CLASS_FORWARD(OptimizationObjective);
  }
  namespace geometric
  {
    class PRMBasic: public ob::Planner{

      public:
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


        struct vertex_associated_vertex_source_t
        {
            typedef boost::vertex_property_tag kind;
        };
        struct vertex_associated_vertex_target_t
        {
            typedef boost::vertex_property_tag kind;
        };
        struct vertex_associated_t_t
        {
            typedef boost::vertex_property_tag kind;
        };


        struct EdgeProperty{
          EdgeProperty(): cost(+dInf), original_cost(+dInf)
          {};
          EdgeProperty(ob::Cost cost_): cost(cost_), original_cost(cost_)
          {};
          void setWeight(double d){
            cost = ob::Cost(d);
          }
          ob::Cost getCost(){
            return cost;
          }
          void setOriginalWeight(){
            cost = original_cost;
          }
          private:
            ob::Cost cost;
            ob::Cost original_cost;
        };

        typedef boost::adjacency_list<
           boost::vecS, boost::vecS, boost::undirectedS,
             boost::property<vertex_state_t, ob::State *,
               boost::property<vertex_total_connection_attempts_t, unsigned long int,
                 boost::property<vertex_successful_connection_attempts_t, unsigned long int,
                   boost::property<boost::vertex_predecessor_t, unsigned long int,
                     boost::property<boost::vertex_rank_t, unsigned long int,
                      boost::property<vertex_associated_vertex_target_t, unsigned long int,
                        boost::property<vertex_associated_vertex_source_t, unsigned long int,
                          boost::property<vertex_associated_t_t, unsigned long int>
                        >
                      >
                     >
                   >
                 >
               >
             >,
             boost::property<boost::edge_weight_t, EdgeProperty>
             //boost::property<boost::edge_weight_t, ob::Cost>>

           >Graph;

        typedef boost::graph_traits<Graph>::vertex_descriptor Vertex;
        typedef boost::graph_traits<Graph>::edge_descriptor Edge;
        typedef std::shared_ptr<NearestNeighbors<Vertex>> RoadmapNeighbors;
        typedef std::function<const std::vector<Vertex> &(const Vertex)> ConnectionStrategy;

      public:

        PRMBasic(const ob::SpaceInformationPtr &si);
        virtual ~PRMBasic() override;

        double GetSamplingDensity();
        ob::PathPtr GetShortestPath();
        ob::PathPtr GetSolutionPath();
        bool hasSolution();

        void Grow(double t);
        ob::PlannerStatus Init(const ob::PlannerTerminationCondition &ptc);

        template <template <typename T> class NN>
        void setNearestNeighbors();

        void getPlannerData(ob::PlannerData &data) const override;

        void setProblemDefinition(const ob::ProblemDefinitionPtr &pdef) override;
        ob::PlannerStatus solve(const ob::PlannerTerminationCondition &ptc) override;

        void clearQuery();
        void clear() override;
        void setup() override;

        Graph g_;

        boost::property_map<Graph, vertex_state_t>::type stateProperty_;
        boost::property_map<Graph, vertex_total_connection_attempts_t>::type totalConnectionAttemptsProperty_;
        boost::property_map<Graph, vertex_successful_connection_attempts_t>::type successfulConnectionAttemptsProperty_;
        boost::property_map<Graph, vertex_associated_vertex_source_t>::type associatedVertexSourceProperty_;
        boost::property_map<Graph, vertex_associated_vertex_target_t>::type associatedVertexTargetProperty_;
        boost::property_map<Graph, vertex_associated_t_t>::type associatedTProperty_;

        ob::OptimizationObjectivePtr opt_;

        std::vector<Vertex> startM_;
        std::vector<Vertex> goalM_;

        void uniteComponents(Vertex m1, Vertex m2);
        bool sameComponent(Vertex m1, Vertex m2);
        ob::Cost bestCost_{+dInf};

        unsigned long int milestoneCount() const
        {
          return boost::num_vertices(g_);
        }
        const Graph &getRoadmap() const
        {
            return g_;
        }
        void checkForSolution(ob::PathPtr &solution);

    protected:

        virtual double Distance(const Vertex a, const Vertex b) const; // standard si->distance
        virtual bool Sample(ob::State *workState); //si->sampler
        virtual bool Connect(const Vertex a, const Vertex b);
        virtual Vertex addMilestone(ob::State *state);

        ob::Cost costHeuristic(Vertex u, Vertex v) const;
        std::vector<ob::State *> xstates;

        ob::ValidStateSamplerPtr sampler_;
        ob::StateSamplerPtr simpleSampler_;
        RoadmapNeighbors nn_;

        boost::disjoint_sets<boost::property_map<Graph, boost::vertex_rank_t>::type,
                             boost::property_map<Graph, boost::vertex_predecessor_t>::type> disjointSets_;
        ConnectionStrategy connectionStrategy_;

        RNG rng_;
        bool addedNewSolution_{false};
        unsigned long int iterations_{0};

        void growRoadmap(const ob::PlannerTerminationCondition &ptc, ob::State *workState);
        void expandRoadmap(const ob::PlannerTerminationCondition &ptc, std::vector<ob::State *> &workStates);

        bool maybeConstructSolution(const std::vector<Vertex> &starts, const std::vector<Vertex> &goals,
                                    ob::PathPtr &solution);
        ob::PathPtr constructSolution(const Vertex &start, const Vertex &goal);

        virtual uint randomBounceMotion(const ob::StateSamplerPtr &sss, 
            const Vertex &v, std::vector<ob::State *> &states) const;

    };
  };
};

