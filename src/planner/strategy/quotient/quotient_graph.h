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

        class VertexInternalState{
          public:
            VertexInternalState() = default;
            VertexInternalState(const VertexInternalState &vis)
            {
              state = vis.state;
              total_connection_attempts = vis.total_connection_attempts;
              successful_connection_attempts = vis.successful_connection_attempts;
              on_shortest_path = vis.on_shortest_path;

              associated_target = vis.associated_target;
              associated_source = vis.associated_source;
              associated_t = vis.associated_t;
              open_neighborhood_distance = vis.open_neighborhood_distance;
              open_neighborhood = vis.open_neighborhood;

              innerApproximationDistance = vis.innerApproximationDistance;
              outerApproximationDistance = vis.outerApproximationDistance;
              start = vis.start;
              goal = vis.goal;
              isSufficient = vis.isSufficient;
              isFeasible = vis.isFeasible;
            }
            ob::State *state{nullptr};
            uint total_connection_attempts{0};
            uint successful_connection_attempts{0};
            bool on_shortest_path{false};

            unsigned long int associated_target{0};
            unsigned long int associated_source{0};
            double associated_t{-1};
            double open_neighborhood_distance{0.0};
            cover::OpenSet *open_neighborhood{nullptr};

            double innerApproximationDistance{0.0};
            double outerApproximationDistance{0.0};
            bool start{false};
            bool goal{false};
            bool isSufficient{false};
            bool isFeasible{false};
        };

        class EdgeInternalState{
          public:
            EdgeInternalState() = default;
            EdgeInternalState(ob::Cost cost_): cost(cost_), original_cost(cost_)
            {};
            EdgeInternalState(const EdgeInternalState &eis)
            {
              cost = eis.cost;
              original_cost = eis.original_cost;
              isSufficient = eis.isSufficient;
            }
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

        typedef boost::adjacency_list<
           boost::vecS, 
           boost::vecS, 
           boost::undirectedS,
           VertexInternalState,
           EdgeInternalState
         > Graph;

        typedef boost::graph_traits<Graph> BGT;
        typedef BGT::vertex_descriptor Vertex;
        typedef BGT::edge_descriptor Edge;
        typedef BGT::vertices_size_type VertexIndex;
        typedef BGT::in_edge_iterator IEIterator;
        typedef BGT::out_edge_iterator OEIterator;
        typedef Vertex* VertexParent;
        typedef VertexIndex* VertexRank;
        typedef std::shared_ptr<NearestNeighbors<Vertex>> RoadmapNeighborsPtr;
        typedef std::function<const std::vector<Vertex> &(const Vertex)> ConnectionStrategy;

      public:

        QuotientGraph(const ob::SpaceInformationPtr &si, Quotient *parent = nullptr);
        ~QuotientGraph();

        virtual uint GetNumberOfVertices() const;
        virtual uint GetNumberOfEdges() const;

        ob::PathPtr GetSolutionPath();

        virtual void Grow(double t);
        virtual bool SampleQuotient(ob::State*) override;
        virtual bool GetSolution(ob::PathPtr &solution) override;
        virtual void getPlannerData(ob::PlannerData &data) const override;
        virtual double GetImportance() const override;
        void Init();
        bool firstRun{true};

        virtual void setup() override;
        virtual void clear() override;
        void clearQuery();
        virtual void ClearVertices();


        template <template <typename T> class NN>
        void setNearestNeighbors();

        virtual void uniteComponents(Vertex m1, Vertex m2);
        bool sameComponent(Vertex m1, Vertex m2);

        bool InsideStartComponent(Vertex v);
        bool InsideStartComponent(Edge e);


        std::map<Vertex, VertexRank> vrank;
        std::map<Vertex, Vertex> vparent;
        boost::disjoint_sets<boost::associative_property_map<std::map<Vertex, VertexRank> >, boost::associative_property_map<std::map<Vertex, Vertex> > > 
          disjointSets_{boost::make_assoc_property_map(vrank), boost::make_assoc_property_map(vparent)};

        ob::Cost bestCost_{+dInf};
        std::vector<Vertex> startM_;
        std::vector<Vertex> goalM_;
        std::vector<Vertex> shortestVertexPath_;
        std::vector<Vertex> startGoalVertexPath_;

        const Graph& GetGraph() const;
        double GetGraphLength() const;
        const RoadmapNeighborsPtr& GetRoadmapNeighborsPtr() const;
        const ConnectionStrategy& GetConnectionStrategy() const;

        virtual void Print(std::ostream& out) const override;
    protected:

        virtual double Distance(const Vertex a, const Vertex b) const; // standard si->distance
        virtual bool Connect(const Vertex a, const Vertex b);
        virtual Vertex addMilestone(ob::State *state);

        virtual Vertex CreateNewVertex(ob::State *state);
        virtual void ConnectVertexToNeighbors(Vertex m);
        ob::Cost costHeuristic(Vertex u, Vertex v) const;

        virtual void growRoadmap(const ob::PlannerTerminationCondition &ptc, ob::State *workState);
        virtual void expandRoadmap(const ob::PlannerTerminationCondition &ptc, std::vector<ob::State *> &workStates);
        ob::PathPtr GetPath(const Vertex &start, const Vertex &goal);
        virtual void RandomWalk(const Vertex &v);

        std::vector<ob::State *> xstates;
        RoadmapNeighborsPtr nn_;
        ConnectionStrategy connectionStrategy_;
        Graph G;
        ob::PathPtr solution_path;
        bool addedNewSolution_{false};
        unsigned long int iterations_{0};
        RNG rng_;
        typedef boost::minstd_rand RNGType;
        RNGType rng_boost;

        double graphLength{0.0};
        uint totalNumberOfSamples{0};

    };
  };
};

