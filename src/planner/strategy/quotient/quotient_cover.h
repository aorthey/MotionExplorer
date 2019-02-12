#pragma once
#include "quotient.h"
#include "elements/plannerdata_vertex_annotated.h"
#include <ompl/datastructures/PDF.h>
#include <ompl/tools/config/SelfConfig.h>
#include <ompl/datastructures/NearestNeighbors.h>
#include <boost/pending/disjoint_sets.hpp>

namespace ob = ompl::base;
namespace og = ompl::geometric;


namespace ompl
{
  namespace geometric
  {
    OMPL_CLASS_FORWARD(QuotientMetric);

    class QuotientCover: public og::Quotient
    {
      typedef og::Quotient BaseT;
    public:

      const uint verbose{0};

      QuotientCover(const ob::SpaceInformationPtr &si, Quotient *parent = nullptr);
      ~QuotientCover(void);
      virtual void clear() override;
      virtual void setup() override;

      typedef int normalized_index_type;
      
      //#######################################################################
      //Configuration = ob::State + Open Neighborhood (Disk)
      //#######################################################################
      class Configuration{
        public:
          Configuration() = default;
          Configuration(const base::SpaceInformationPtr &si);
          Configuration(const base::SpaceInformationPtr &si, const ob::State *state_);
          ~Configuration();
          double GetRadius() const;
          void SetRadius(double radius);
          void SetOuterRadius(double radius);
          void Remove(const base::SpaceInformationPtr &si);
          void Clear();
          void SetPDFElement(void *element_);
          void* GetPDFElement();
          void SetNecessaryPDFElement(void *element_);
          void* GetNecessaryPDFElement();
          void SetConnectivityPDFElement(void *element_);
          void* GetConnectivityPDFElement();
          double GetImportance() const;
          double GetGoalDistance() const;
          ob::State* GetInwardPointingConfiguration() const;
          void UpdateRiemannianCenterOfMass(og::QuotientCover*, Configuration*);

          friend std::ostream& operator<< (std::ostream& out, const ompl::geometric::QuotientCover::Configuration&);

          double goal_distance{+dInf};
          uint number_attempted_expansions{0};
          uint number_successful_expansions{0};

          base::State *state{nullptr};
          base::State *riemannian_center_of_mass{nullptr}; //geometric mean constrained to neighborhood boundary
          uint number_of_neighbors{0}; //counter for incremental computation of riemannian center of mass (RCoM)

          bool isSufficientFeasible{false};
          void *pdf_element;
          void *pdf_necessary_element;
          void *pdf_connectivity_element;

          bool isStart{false};
          bool isGoal{false};

          normalized_index_type index{-1}; //in [0,num_vertices(graph)]

          //#####################################################################
          //Neighborhood Computations
          //#####################################################################
          double openNeighborhoodRadius{0.0}; //might be L1 or L2 radius
          double openNeighborhoodOuterRadius{0.0}; //might be L1 or L2 radius

      };

      class EdgeInternalState{
        public:
          EdgeInternalState() = default;
          EdgeInternalState(ob::Cost cost_): cost(cost_)
          {};
          EdgeInternalState(double w): cost(w)
          {};
          EdgeInternalState(const EdgeInternalState &eis)
          {
            cost = eis.cost;
            isSufficient = eis.isSufficient;
          }
          void setWeight(double d){
            cost = ob::Cost(d);
          }
          double getWeight() const{
            return cost.value();
          }
          ob::Cost getCost() const{
            return cost;
          }
        private:
          ob::Cost cost{+dInf};
          bool isSufficient{false};
      };
      virtual void Print(std::ostream& out) const override;
      void Print(const Configuration *q, bool stopOnError=true) const;
    private:

      struct GraphBundle{
        // put PDFs and neighborhood structures here (so they get copied when
        // copying the graph)
        std::string name{"graph"};
        ob::SpaceInformationPtr Q1;
      };

      typedef boost::adjacency_list<
         boost::setS, //do not change to vecS, otherwise vertex indices are not stable after removal
         boost::setS, 
         boost::undirectedS,
         Configuration*,
         EdgeInternalState,
         GraphBundle
       > Graph;
      friend std::ostream& operator<< (std::ostream& out, const Graph& graph);


    protected:
      typedef boost::graph_traits<Graph> BGT;
      typedef BGT::vertex_descriptor Vertex;
      typedef BGT::edge_descriptor Edge;
      typedef BGT::vertices_size_type VertexIndex;
      typedef BGT::in_edge_iterator IEIterator;
      typedef BGT::out_edge_iterator OEIterator;
      typedef Vertex* VertexParent;
      typedef VertexIndex* VertexRank;
      //typedef std::function<const std::vector<Vertex> &(const Vertex)> ConnectionStrategy;

      
      //keep manual track of indices, because we sometimes need to remove
      //vertices. Vertex (the number assigned to a boost::graph vertex) might
      //need to be deleted. But astar and similar algorithms need a number
      //between [0, num_vertices(graph)]. We therefore need to map that to an
      //integer we call normalized_index_type
      typedef std::map<typename boost::graph_traits<Graph>::vertex_descriptor, normalized_index_type> VertexToNormalizedIndexMap;
      VertexToNormalizedIndexMap vertexToNormalizedIndexStdMap;
      boost::associative_property_map<VertexToNormalizedIndexMap> vertexToNormalizedIndex{vertexToNormalizedIndexStdMap};

      typedef std::map<normalized_index_type, typename boost::graph_traits<Graph>::vertex_descriptor> NormalizedIndexToVertexMap;
      NormalizedIndexToVertexMap normalizedIndexToVertexStdMap;
      boost::associative_property_map<NormalizedIndexToVertexMap> normalizedIndexToVertex{normalizedIndexToVertexStdMap};
      normalized_index_type index_ctr{0};

    public:

      typedef std::shared_ptr<NearestNeighbors<Configuration*>> NearestNeighborsPtr;
      typedef ompl::PDF<Configuration*> PDF;
      typedef PDF::Element PDF_Element;
      //#######################################################################
      //Configuration Create, Remove, Add 
      //#######################################################################
      virtual Vertex AddConfigurationToCover(Configuration *q);
      virtual Vertex AddConfigurationToCoverGraph(Configuration *q);
      virtual void AddConfigurationToPDF(Configuration *q);
      virtual void RemoveConfigurationFromCover(Configuration *q);

      void AddEdge(Configuration *q_from, Configuration *q_to);
      bool EdgeExists(Configuration *q_from, Configuration *q_to);
      int GetNumberOfEdges(Configuration *q);

      bool ComputeNeighborhood(Configuration *q, bool verbose = false);
      void RewireConfiguration(Configuration *q);

      std::vector<Vertex> GetCoverPath(const Vertex& v_source, const Vertex& v_sink);
      std::vector<const Configuration*> GetCoverPath(const Configuration *q_source, const Configuration *q_sink);

      //#######################################################################
      //Sampling Sample{Structure}{Substructure}
      //#######################################################################

      void SampleGoal(Configuration*);
      void SampleUniform(Configuration*);
      virtual void SampleUniformQuotientCover(ob::State *state);
      Configuration* SampleNeighborhoodBoundary(Configuration *q);
      Configuration* SampleNeighborhoodBoundaryUniformNear(Configuration *q_center, const Configuration* q_near, const double radius);

      //#######################################################################
      //Misc
      //#######################################################################
      virtual void Grow(double t) override = 0;
      virtual bool GetSolution(ob::PathPtr &solution) override;
      virtual double GetImportance() const override;

      void ProjectConfigurationOntoNeighborhoodBoundary(const Configuration *q_center, Configuration* q_projected);
      Configuration* NearestConfigurationOnBoundary(Configuration *q_center, const Configuration* q_outside);
      Configuration* GetOutwardPointingConfiguration(Configuration *q);

      void Init();
      void Test();

      //#######################################################################
      //Neighborhood Set Computations
      //#######################################################################
      bool IsConfigurationInsideNeighborhood(Configuration *q, Configuration *qn);
      bool IsNeighborhoodInsideNeighborhood(Configuration *lhs, Configuration *rhs);
      bool IsConfigurationInsideCover(Configuration *q);
      Configuration* NearestNeighborhood(const Configuration *q) const;
      Configuration* NearestConfiguration(const Configuration *q) const;

      virtual void getPlannerData(base::PlannerData &data) const override;

    protected:
      PlannerDataVertexAnnotated getAnnotatedVertex(Vertex vertex) const;
      PlannerDataVertexAnnotated getAnnotatedVertex(ob::State* state, double radius, bool sufficient) const;
      //#######################################################################
      RNG rng_;
      const double minimum_neighborhood_radius{1e-3}; //minimum allowed radius, otherwise configuration is considered INVALID 

      double totalVolumeOfCover{0.0};
      bool isConnected{false};
      bool saturated{false}; //if space is saturated, then the whole free space has been found
      bool firstRun{true};

      Graph graph;
      NearestNeighborsPtr nearest_neighborhood{nullptr};
      NearestNeighborsPtr nearest_configuration{nullptr};
      PDF pdf_necessary_configurations;
      PDF pdf_all_configurations;
      std::vector<Vertex> shortest_path_start_goal;
      std::vector<Vertex> shortest_path_start_goal_necessary_vertices;

    private:
      void RerouteEdgesFromTo(Configuration *q_from, Configuration *q_to);
      Configuration *q_start{nullptr};
      Configuration *q_goal{nullptr};
      QuotientMetricPtr metric{nullptr};
      Vertex v_start;
      Vertex v_goal;

    public:

      const QuotientMetricPtr& GetMetric();
      void SetMetric(const std::string&); //arguments: euclidean, shortestpath

      Configuration* GetStartConfiguration() const;
      Configuration* GetGoalConfiguration() const;
      const PDF& GetPDFNecessaryConfigurations() const;
      const PDF& GetPDFAllConfigurations() const;
      const NearestNeighborsPtr& GetNearestNeighborsCover() const;
      const NearestNeighborsPtr& GetNearestNeighborsVertex() const;


    };
  }
}


