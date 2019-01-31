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
      //Configuration = ob::State + Open Neighborhood
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
          void UpdateRiemannianCenterOfMass(ob::SpaceInformationPtr si, Configuration* q_new);

          friend std::ostream& operator<< (std::ostream& out, const ompl::geometric::QuotientCover::Configuration&);

          double goal_distance{+dInf};
          uint number_attempted_expansions{0};
          uint number_successful_expansions{0};

          base::State *state{nullptr};
          Configuration *coset{nullptr}; //the underlying coset this Vertex belongs to (on the quotient-space)
          Configuration *parent_neighbor{nullptr}; //the configuration from which this configuration has been spawned
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

      typedef boost::graph_traits<Graph> BGT;
      typedef BGT::vertex_descriptor Vertex;
      typedef BGT::edge_descriptor Edge;
      typedef BGT::vertices_size_type VertexIndex;
      typedef BGT::in_edge_iterator IEIterator;
      typedef BGT::out_edge_iterator OEIterator;
      typedef Vertex* VertexParent;
      typedef VertexIndex* VertexRank;
      //typedef std::function<const std::vector<Vertex> &(const Vertex)> ConnectionStrategy;
      typedef std::shared_ptr<NearestNeighbors<Configuration*>> NearestNeighborsPtr;
      typedef ompl::PDF<Configuration*> PDF;
      typedef PDF::Element PDF_Element;

      
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

      //#######################################################################
      //Configuration Create, Remove, Add 
      //#######################################################################
      Configuration* CreateConfigurationFromStateAndCoset(const ob::State *state, Configuration *q_coset);
      virtual Vertex AddConfigurationToCover(Configuration *q);
      virtual Vertex AddConfigurationToCoverGraph(Configuration *q);
      void RerouteEdgesFromTo(Configuration *q_from, Configuration *q_to);

      virtual void RemoveConfigurationFromCover(Configuration *q);
      void AddEdge(Configuration *q_from, Configuration *q_to);
      bool EdgeExists(Configuration *q_from, Configuration *q_to);
      int GetNumberOfEdges(Configuration *q);
      Configuration* GetOutwardPointingConfiguration(Configuration *q);

      virtual void AddConfigurationToPDF(Configuration *q);
      bool IsConfigurationInsideCover(Configuration *q);
      void RemoveConfigurationsFromCoverCoveredBy(Configuration *q);
      bool ComputeNeighborhood(Configuration *q, bool verbose = false);
      void RewireConfiguration(Configuration *q);
      void CheckConfigurationIsOnBoundary(Configuration *q_boundary, Configuration *q);

      void Init();
      bool firstRun{true};
      void Test();

      std::vector<const Configuration*> GetInterpolationPath(const Configuration *q_from, const Configuration *q_to);
      std::vector<Vertex> GetCoverPath(const Vertex& v_source, const Vertex& v_sink);
      std::vector<const Configuration*> GetCoverPath(const Configuration *q_source, const Configuration *q_sink);

      //#######################################################################
      //Sampling Sample{Structure}{Substructure}
      //#######################################################################

      void SampleGoal(Configuration*);
      void SampleUniform(Configuration*);
      virtual Configuration* SampleUniformQuotientCover(ob::State *state);
      Configuration* SampleNeighborhoodBoundary(Configuration *q);
      Configuration* SampleNeighborhoodBoundaryUniformNear(Configuration *q_center, const Configuration* q_near, const double radius);

      //#######################################################################
      //Connect strategies
      //#######################################################################
      virtual void Grow(double t) override = 0;
      bool GetSolution(ob::PathPtr &solution) override;
      virtual double GetImportance() const override;

      bool Interpolate(const Configuration*, Configuration*);
      bool Interpolate(const Configuration* q_from, const Configuration* q_to, Configuration* q_output);

      bool Interpolate(const Configuration*, const Configuration*, double step_size, Configuration*);
      bool InterpolateQ1(const Configuration*, const Configuration*, double step_size, Configuration*);

      void InterpolateUntilNeighborhoodBoundary(const Configuration *q_center, const Configuration *q_desired, Configuration *q_out);

      void ProjectConfigurationOntoNeighborhoodBoundary(const Configuration *q_center, Configuration* q_projected);
      Configuration* NearestConfigurationOnBoundary(Configuration *q_center, const Configuration* q_outside);

      //#######################################################################
      //Neighborhood Set Computations
      //#######################################################################
      bool IsConfigurationInsideNeighborhood(Configuration *q, Configuration *qn);
      std::vector<Configuration*> GetIntersectingNeighborhoodConfigurations(Configuration *q);

      std::vector<Configuration*> GetConfigurationsInsideNeighborhood(Configuration *q);

      bool IsNeighborhoodInsideNeighborhood(Configuration *lhs, Configuration *rhs);
      void GetCosetFromQuotientSpace(Configuration *q);

      //#######################################################################
      //Cover Algorithms
      //#######################################################################

      Configuration* Nearest(Configuration *q) const;
      virtual void getPlannerData(base::PlannerData &data) const override;

    protected:
      PlannerDataVertexAnnotated getAnnotatedVertex(Vertex vertex) const;
      PlannerDataVertexAnnotated getAnnotatedVertex(ob::State* state, double radius, bool sufficient) const;
      //#######################################################################
      RNG rng_;
      const double minimum_neighborhood_radius{1e-3}; //minimum allowed radius, otherwise configuration is considered INVALID 

      double totalVolumeOfCover{0.0};
      bool isConnected{false};
      bool saturated{false}; //if space is saturated, then we the whole free space has been found

      Graph graph;
      NearestNeighborsPtr nearest_neighborhood{nullptr};
      NearestNeighborsPtr nearest_vertex{nullptr};
      Configuration *q_start{nullptr};
      Configuration *q_goal{nullptr};
      Vertex v_start;
      Vertex v_goal;
      PDF pdf_necessary_configurations;
      PDF pdf_all_configurations;
      std::vector<Vertex> shortest_path_start_goal;
      std::vector<Vertex> shortest_path_start_goal_necessary_vertices;

      QuotientMetricPtr metric{nullptr};
    public:

      const QuotientMetricPtr& GetMetric();

      Configuration* GetStartConfiguration() const;
      Configuration* GetGoalConfiguration() const;
      const PDF& GetPDFNecessaryConfigurations() const;
      const PDF& GetPDFAllConfigurations() const;
      const NearestNeighborsPtr& GetNearestNeighborsCover() const;
      const NearestNeighborsPtr& GetNearestNeighborsVertex() const;

      virtual void Print(std::ostream& out) const override;
      virtual void Print(const Configuration *q, bool stopOnError=true) const;

    };
  }
}


