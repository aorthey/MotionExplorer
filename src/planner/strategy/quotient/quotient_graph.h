#pragma once

#include "quotient.h"
#include "planner/cspace/cover/open_set.h"
#include <stack>
#include <ompl/geometric/planners/PlannerIncludes.h>
#include <ompl/datastructures/NearestNeighbors.h>
#include <ompl/base/Cost.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <KrisLibrary/math/infnan.h> //dInf, fInf, IsNaN(x)
#include <boost/pending/disjoint_sets.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/random.hpp> 
#include <boost/graph/subgraph.hpp>
#include <boost/graph/properties.hpp>
#include <boost/random/linear_congruential.hpp>
#include <boost/random/variate_generator.hpp>
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
    class QuotientGraph: public og::Quotient{

        typedef og::Quotient BaseT;
      public:

        typedef unsigned long int VertexIndexType;

        class VertexInternalState{
          public:
            VertexInternalState() = default;
            VertexInternalState(const VertexInternalState &vis)
            {
              //state = si_->cloneState(vis.state);
              state = vis.state;
              total_connection_attempts = vis.total_connection_attempts;
              successful_connection_attempts = vis.successful_connection_attempts;
              on_shortest_path = vis.on_shortest_path;
              associated_target = vis.associated_target;
              associated_source = vis.associated_source;
              associated_t = vis.associated_t;
              open_neighborhood_distance = vis.open_neighborhood_distance;
              open_neighborhood = vis.open_neighborhood;
            }
            ob::State *state{nullptr};
            uint total_connection_attempts{0};
            uint successful_connection_attempts{0};
            bool on_shortest_path{false};
            unsigned long int associated_target{0};
            unsigned long int associated_source{0};
            double associated_t{-1};
            double open_neighborhood_distance{0};
            bool start{false};
            bool goal{false};
            cover::OpenSet *open_neighborhood{nullptr};
        };

        class EdgeInternalState{
          public:
            EdgeInternalState() = default;
            EdgeInternalState(ob::Cost cost_): cost(cost_), original_cost(cost_)
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
            ob::Cost cost{+dInf};
            ob::Cost original_cost{+dInf};
            bool isSufficient{false};
        };

        // typedef boost::property<boost::vertex_index_t, std::size_t, VertexInternalState> vertex_prop;
        // typedef boost::property<boost::edge_index_t, std::size_t, EdgeInternalState> edge_prop;

        typedef boost::adjacency_list<
           boost::vecS, 
           boost::vecS, 
           boost::undirectedS,
           VertexInternalState,
           EdgeInternalState
         > Graph;

        typedef boost::graph_traits<Graph>::vertex_descriptor Vertex;
        typedef boost::graph_traits<Graph>::edge_descriptor Edge;
        typedef boost::graph_traits<Graph>::vertices_size_type VertexIndex;
        typedef boost::graph_traits<Graph>::in_edge_iterator IEIterator;
        typedef Vertex* VertexParent;
        typedef VertexIndex* VertexRank;

        typedef std::shared_ptr<NearestNeighbors<Vertex>> RoadmapNeighbors;
        typedef std::function<const std::vector<Vertex> &(const Vertex)> ConnectionStrategy;

      public:

        QuotientGraph(const ob::SpaceInformationPtr &si, Quotient *parent = nullptr);
        ~QuotientGraph();

        virtual uint GetNumberOfVertices() const override;
        virtual uint GetNumberOfEdges() const override;
        ob::PathPtr GetShortestPath();
        ob::PathPtr GetSolutionPath();

        virtual void Grow(double t);
        virtual void Init() override;
        virtual bool SampleGraph(ob::State *q_random_graph) override;

        // template <template <typename T> class NN>
        // void setNearestNeighbors();

        void getPlannerData(ob::PlannerData &data) const override;

        virtual void setup() override;

        virtual void clear() override;
        void clearQuery();
        virtual void ClearVertices();


        virtual void uniteComponents(Vertex m1, Vertex m2);
        bool sameComponent(Vertex m1, Vertex m2);

        bool InsideStartComponent(Vertex v);
        bool InsideStartComponent(Edge e);

        virtual void CheckForSolution(ob::PathPtr &solution) override;

        std::map<Vertex, VertexRank> vrank;
        std::map<Vertex, Vertex> vparent;
        boost::disjoint_sets<boost::associative_property_map<std::map<Vertex, VertexRank> >, boost::associative_property_map<std::map<Vertex, Vertex> > > 
          disjointSets_{boost::make_assoc_property_map(vrank), boost::make_assoc_property_map(vparent)};

        ob::Cost bestCost_{+dInf};
        ob::OptimizationObjectivePtr opt_;
        std::vector<Vertex> startM_;
        std::vector<Vertex> goalM_;
        std::vector<Vertex> shortestVertexPath_;

        const Graph& GetGraph() const;

    protected:

        Graph G;
        ob::PathPtr solution_path;

        virtual double Distance(const Vertex a, const Vertex b) const; // standard si->distance
        virtual bool Connect(const Vertex a, const Vertex b);
        virtual Vertex addMilestone(ob::State *state);

        virtual Vertex CreateNewVertex(ob::State *state);
        virtual void ConnectVertexToNeighbors(Vertex m);

        ob::Cost costHeuristic(Vertex u, Vertex v) const;
        std::vector<ob::State *> xstates;

        RoadmapNeighbors nn_;

        ConnectionStrategy connectionStrategy_;

        RNG rng_;
        typedef boost::minstd_rand RNGType;
        RNGType rng_boost;

        bool addedNewSolution_{false};
        unsigned long int iterations_{0};

        virtual void growRoadmap(const ob::PlannerTerminationCondition &ptc, ob::State *workState);
        virtual void expandRoadmap(const ob::PlannerTerminationCondition &ptc, std::vector<ob::State *> &workStates);

        ob::PathPtr GetSolutionPath(const Vertex &start, const Vertex &goal);

        virtual void RandomWalk(const Vertex &v);

    };
  };
};

