#pragma once

// #include "planner/strategy/quotientgraph/quotient_graph.h"
#include "planner/strategy/quotient/quotient.h"
#include <ompl/datastructures/NearestNeighbors.h>
#include <ompl/util/RandomNumbers.h>
#include <boost/graph/subgraph.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/random.hpp> 
#include <boost/random/linear_congruential.hpp>
#include <boost/random/variate_generator.hpp>

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace ompl
{
  namespace geometric
  {
    class QuotientSubGraph: public og::Quotient{

        typedef og::Quotient BaseT;
      public:
        typedef int normalized_index_type;

        class Configuration{
          public:
            Configuration() = delete;
            Configuration(const ob::SpaceInformationPtr &si);
            Configuration(const ob::SpaceInformationPtr &si, const ob::State *state_);
            ob::State *state{nullptr};
            bool on_shortest_path{false};

            void *pdf_element;
            void SetPDFElement(void *element_)
            {
              pdf_element = element_;
            }
            void* GetPDFElement()
            {
              return pdf_element;
            }
            void Remove(const base::SpaceInformationPtr &si)
            {
              if(state) si->freeState(state);
            }
            bool isStart{false};
            bool isGoal{false};
            normalized_index_type index{-1}; //in [0,num_vertices(graph)]
        };

        class EdgeInternalState{
          public:
            EdgeInternalState() = default;
            EdgeInternalState(ob::Cost cost_): cost(cost_)
            {};
            EdgeInternalState(const EdgeInternalState &eis)
            {
              cost = eis.cost;
            }
            void setWeight(double d){
              cost = ob::Cost(d);
            }
            ob::Cost getCost(){
              return cost;
            }
          private:
            ob::Cost cost{+dInf};
        };
        struct GraphBundle{
          std::string name{"quotient_subgraph"};
        };
        typedef boost::subgraph<
          boost::adjacency_list<
           boost::vecS, 
           boost::vecS, 
           boost::undirectedS,
           Configuration*,
           boost::property<boost::edge_index_t, int, EdgeInternalState>
          >
         >SubGraph;

        typedef boost::graph_traits<SubGraph> BGT;
        typedef BGT::vertex_descriptor Vertex;
        typedef BGT::edge_descriptor Edge;
        typedef BGT::vertices_size_type VertexIndex;
        typedef BGT::in_edge_iterator IEIterator;
        typedef BGT::out_edge_iterator OEIterator;
        typedef Vertex* VertexParent;
        typedef VertexIndex* VertexRank;
        typedef std::shared_ptr<NearestNeighbors<Configuration*>> RoadmapNeighborsPtr;

      public:

        QuotientSubGraph(const ob::SpaceInformationPtr &si, Quotient *parent = nullptr);
        ~QuotientSubGraph();
        // void Rewire(Vertex &v);
        // void Rewire();
        const Configuration* Nearest(const Configuration *s) const;

        virtual void Grow(double t) = 0;
        virtual bool SampleQuotient(ob::State*) override;
        virtual bool GetSolution(ob::PathPtr &solution) override;
        virtual void getPlannerData(ob::PlannerData &data) const override;
        virtual double GetImportance() const override;
        virtual bool Sample(ob::State *q_random) override;

        virtual void DeleteConfiguration(Configuration *q);
        virtual Vertex AddConfiguration(Configuration *q);
        void AddConfigurationConditionalSparse(const Vertex &v);
        void AddEdge(const Configuration* q1, const Configuration* q2);
        virtual double Distance(const Configuration* a, const Configuration* b) const; // standard si->distance

        virtual void setup() override;
        virtual void clear() override;
        void Init();

        std::vector<const Configuration*> GetPathOnGraph(const Configuration *q_source, const Configuration *q_sink);
        std::vector<Vertex> GetPathOnGraph(const Vertex& v_source, const Vertex& v_sink);
        std::vector<Vertex> GetPathOnGraph(const Vertex& v_source, const Vertex& v_intermediate, const Vertex& v_sink);

        //Vertex to Index
        typedef std::map<typename BGT::vertex_descriptor, normalized_index_type> VertexToIndexMap;
        VertexToIndexMap vertexToIndexStdMap;
        boost::associative_property_map<VertexToIndexMap> vertexToIndex{vertexToIndexStdMap};

        //Index to Vertex
        typedef std::map<normalized_index_type, typename BGT::vertex_descriptor> IndexToVertexMap;
        IndexToVertexMap indexToVertexStdMap;
        boost::associative_property_map<IndexToVertexMap> indexToVertex{indexToVertexStdMap};

        normalized_index_type index_ctr{0};

    protected:

        SubGraph graphSparse_;
        SubGraph graphDense_;
        RoadmapNeighborsPtr nearestSparse_;
        RoadmapNeighborsPtr nearestDense_;

        std::vector<Vertex> shortest_path_start_goal;
        Configuration *q_start;
        Configuration *q_goal;
        Vertex v_start;
        Vertex v_goal;
        RNG rng_;
        typedef boost::minstd_rand RNGType;
        RNGType rng_boost;

    };
  };
};


