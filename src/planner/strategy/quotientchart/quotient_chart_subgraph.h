#pragma once
#include "quotient_chart.h"
#include "elements/plannerdata_vertex_annotated.h"
#include <ompl/datastructures/PDF.h>
#include <ompl/tools/config/SelfConfig.h>
#include <ompl/datastructures/NearestNeighbors.h>
#include <boost/pending/disjoint_sets.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/subgraph.hpp>
#include <boost/graph/random.hpp> 
#include <boost/random/linear_congruential.hpp>
#include <boost/random/variate_generator.hpp>

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace ompl
{
  namespace geometric
  {
    class QuotientChartSubGraph: public og::QuotientChart
    {
      typedef og::QuotientChart BaseT;
    public:

      const uint verbose{0};

      QuotientChartSubGraph(const ob::SpaceInformationPtr &si, Quotient *parent = nullptr);
      ~QuotientChartSubGraph(void);
      virtual void clear() override;
      virtual void setup() override;

      typedef int normalized_index_type;

      class Configuration{
        public:
          Configuration() = default;
          Configuration(const ob::SpaceInformationPtr &si);
          Configuration(const ob::SpaceInformationPtr &si, const ob::State *state_);
          ob::State *state{nullptr};
          uint total_connection_attempts{0};
          uint successful_connection_attempts{0};
          bool on_shortest_path{false};
          std::vector<int> components;

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
          bool IsMemberOf(int k)
          {
            return std::find(components.begin(), components.end(), k) != components.end();
          }

          unsigned long int associated_target{0};
          unsigned long int associated_source{0};
          double associated_t{-1};

          bool isStart{false};
          bool isGoal{false};
          bool isFeasible{false};

          normalized_index_type index{-1}; //in [0,num_vertices(graph)]
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
          std::vector<int> components;
          bool IsMemberOf(int k)
          {
            return std::find(components.begin(), components.end(), k) != components.end();
          }
        private:
          ob::Cost cost{+dInf};
          ob::Cost original_cost{+dInf};
      };

      typedef boost::subgraph<
        boost::adjacency_list<
         boost::vecS, 
         boost::vecS, 
         boost::undirectedS,
         //boost::property<boost::vertex_index_t, int, Configuration*>,
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
      //typedef std::function<const std::vector<Configuration*> &(const Configuration*)> ConnectionStrategy;
      typedef ompl::PDF<Configuration*> PDF;
      typedef PDF::Element PDF_Element;

      //Vertex to Index
      typedef std::map<typename BGT::vertex_descriptor, normalized_index_type> VertexToIndexMap;
      VertexToIndexMap vertexToIndexStdMap;
      boost::associative_property_map<VertexToIndexMap> vertexToIndex{vertexToIndexStdMap};

      //Index to Vertex
      typedef std::map<normalized_index_type, typename BGT::vertex_descriptor> IndexToVertexMap;
      IndexToVertexMap indexToVertexStdMap;
      boost::associative_property_map<IndexToVertexMap> indexToVertex{indexToVertexStdMap};

      normalized_index_type index_ctr{0};

    public:

      void UpdateIndex(const Vertex v);
      virtual uint GetNumberOfVertices() const;
      virtual uint GetNumberOfEdges() const;

      //virtual bool SampleQuotient(ob::State*) override;
      virtual bool GetSolution(ob::PathPtr &solution) override;

      void Init();

      void clearQuery();

      template <template <typename T> class NN>
      void setNearestNeighbors();

      Configuration* Nearest(Configuration *q) const;
      std::vector<const QuotientChartSubGraph::Configuration*> 
        GetPathOnGraph(const Configuration *q_source, const Configuration *q_sink);
      std::vector<QuotientChartSubGraph::Vertex> 
        GetPathOnGraph(const Vertex& v_source, const Vertex& v_sink);
      std::vector<QuotientChartSubGraph::Vertex> 
        GetPathOnGraph(const Vertex& v_source, const Vertex& v_intermediate, const Vertex& v_sink);

      virtual std::vector<int> VertexBelongsToComponents(const Vertex &v);
      virtual bool FoundNewComponent() override;

      // std::map<Vertex, VertexRank> vrank;
      // std::map<Vertex, Vertex> vparent;
      // boost::disjoint_sets<boost::associative_property_map<std::map<Vertex, VertexRank> >, boost::associative_property_map<std::map<Vertex, Vertex> > > 
      //   disjointSets_{boost::make_assoc_property_map(vrank), boost::make_assoc_property_map(vparent)};

      ob::Cost bestCost_{+dInf};
      Configuration *q_start;
      Configuration *q_goal;
      Vertex v_start;
      Vertex v_goal;
      const ob::State *s_goal;
      std::vector<Vertex> shortestVertexPath_;
      std::vector<Vertex> startGoalVertexPath_;

      virtual void getPlannerDataAnnotated(ob::PlannerData &data) const;
      PlannerDataVertexAnnotated getAnnotatedVertex(const Vertex &v) const;

      const SubGraph& GetGraph() const;
      virtual SubGraph& GetSubGraphComponent(int k_component);

      double GetGraphLength() const;
      const RoadmapNeighborsPtr& GetRoadmapNeighborsPtr() const;

      virtual void Print(std::ostream& out) const override;
      void PrintConfiguration(const Configuration*) const;
      void PrintGraph() const;

      virtual void CopyChartFromSibling( QuotientChart *sibling, uint k ) override;
      void ExtendGraphOneStep();
      virtual void Grow(double t) override;
      std::vector<ob::State*> PathToStates(std::vector<Vertex> &path);
      virtual bool IsPathVisible(std::vector<ob::State*> &s1, std::vector<ob::State*> &s2);
  protected:

      virtual double Distance(const Configuration* a, const Configuration* b) const; // standard si->distance
      virtual Vertex AddConfiguration(const ob::State *s);
      std::vector<Vertex> shortest_path_start_goal;
      virtual Edge AddEdge(const Configuration *a, const Configuration *b);

      ob::Cost costHeuristic(Vertex u, Vertex v) const;
      std::vector<ob::State*> GetShortestPath();

      ob::PathPtr GetPath(const Vertex &start, const Vertex &goal);
      void Rewire();
      void Rewire(Vertex &v);
      bool isConnected{false};

      ob::Goal *goal;
      double maxDistance{.0};
      double goalBias{.05};
      Configuration *q_random{nullptr};

      RoadmapNeighborsPtr nearest_configuration;
      SubGraph graph;
      ob::PathPtr solution_path;
      bool addedNewSolution_{false};
      unsigned long int iterations_{0};
      RNG rng_;
      typedef boost::minstd_rand RNGType;
      RNGType rng_boost;

      double graphLength{0.0};

    };
  }
}


