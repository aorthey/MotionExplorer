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
    class QuotientCover: public og::Quotient
    {
      typedef og::Quotient BaseT;
    public:

      const uint verbose{0};

      QuotientCover(const ob::SpaceInformationPtr &si, Quotient *parent = nullptr);
      ~QuotientCover(void);
      virtual void clear() override;
      virtual void setup() override;

      typedef int vertex_index_type;
      

      //#######################################################################
      //Configuration
      //#######################################################################
      class Configuration{
        public:
          Configuration() = default;
        Configuration(const base::SpaceInformationPtr &si) : state(si->allocState())
        {}
        Configuration(const base::SpaceInformationPtr &si, const ob::State *state_) : state(si->cloneState(state_))
        {}
        ~Configuration(){};
        double GetRadius() const
        {
          return openNeighborhoodRadius;
        }
        void SetRadius(double radius)
        {
          openNeighborhoodRadius = radius;
        }
        void SetOuterRadius(double radius)
        {
          openNeighborhoodOuterRadius = radius;
        }
        void Remove(const base::SpaceInformationPtr &si)
        {
          si->freeState(state);
        }
        void Clear()
        {
          number_attempted_expansions = 0;
          number_successful_expansions = 0;

          parent_neighbor = nullptr;
          isSufficientFeasible = false;
          pdf_element = nullptr;
          pdf_necessary_element = nullptr;
          pdf_connectivity_element = nullptr;

          index = -1;
          goal_distance = 0.0;
        }
        void SetPDFElement(void *element_)
        {
          pdf_element = element_;
        }
        void* GetPDFElement()
        {
          return pdf_element;
        }
        void SetNecessaryPDFElement(void *element_)
        {
          pdf_necessary_element = element_;
        }
        void* GetNecessaryPDFElement()
        {
          return pdf_necessary_element;
        }
        void SetConnectivityPDFElement(void *element_)
        {
          pdf_connectivity_element = element_;
        }
        void* GetConnectivityPDFElement()
        {
          return pdf_connectivity_element;
        }

        double GetImportance() const
        {
          //return openNeighborhoodRadius + 1.0/goal_distance+1;
          double d = GetRadius();
          return d;
          //return ((double)number_successful_expansions+1)/((double)number_attempted_expansions+2);
          //return 1.0/((double)number_attempted_expansions+1);
        }
        double GetGoalDistance() const
        {
          return goal_distance;
        }

        double goal_distance{0.0};
        uint number_attempted_expansions{0};
        uint number_successful_expansions{0};

        base::State *state{nullptr};
        Configuration *coset{nullptr}; //the underlying coset this Vertex elongs to (on the quotient-space)
        Configuration *parent_neighbor{nullptr};

        bool isSufficientFeasible{false};
        void *pdf_element;
        void *pdf_necessary_element;
        void *pdf_connectivity_element;

        bool isStart{false};
        bool isGoal{false};

        vertex_index_type index{-1};

        //#####################################################################
        //Neighborhood Computations
        //#####################################################################
        double openNeighborhoodRadius{0.0}; //might be L1 or L2 radius
        double openNeighborhoodOuterRadius{0.0}; //might be L1 or L2 radius

        friend std::ostream& operator<< (std::ostream& out, const ompl::geometric::QuotientCover::Configuration&);
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
         //boost::property<boost::edge_index_t,int, EdgeInternalState> 
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
      //vertices
      typedef std::map<typename boost::graph_traits<Graph>::vertex_descriptor, vertex_index_type> VertexToIndexMap;
      VertexToIndexMap vertexToIndexStdMap;
      boost::associative_property_map<VertexToIndexMap> vertexToIndex{vertexToIndexStdMap};

      typedef std::map<vertex_index_type, typename boost::graph_traits<Graph>::vertex_descriptor> IndexToVertexMap;
      IndexToVertexMap indexToVertexStdMap;
      boost::associative_property_map<IndexToVertexMap> indexToVertex{indexToVertexStdMap};
      vertex_index_type index_ctr{0};

    protected:

      //#######################################################################
      //Configuration Create, Remove, Add 
      //#######################################################################

      Configuration* CreateConfigurationFromStateAndCoset(const ob::State *state, Configuration *q_coset);

      virtual Vertex AddConfigurationToCover(Configuration *q);
      Vertex AddConfigurationToCoverWithoutAddingEdges(Configuration *q);

      void RemoveConfigurationFromCover(Configuration *q);
      void AddEdge(Configuration *q_from, Configuration *q_to);
      bool EdgeExists(Configuration *q_from, Configuration *q_to);

      virtual void AddConfigurationToPDF(Configuration *q);
      bool IsConfigurationInsideCover(Configuration *q);
      void RemoveConfigurationsFromCoverCoveredBy(Configuration *q);
      bool ComputeNeighborhood(Configuration *q, bool verbose = false);
      void RewireConfiguration(Configuration *q);
      void CheckConfigurationIsOnBoundary(Configuration *q_boundary, Configuration *q);
      uint GetLargestNeighborhoodIndex(const std::vector<Configuration*> &q_children);

      //#######################################################################
      //Distance Computations
      //#######################################################################
      double DistanceQ1(const Configuration *q_from, const Configuration *q_to);
      double DistanceX1(const Configuration *q_from, const Configuration *q_to);

      double DistanceConfigurationConfiguration(const Configuration *q_from, const Configuration *q_to);
      //Note: this is a pseudometric: invalidates second axiom of metric : d(x,y) = 0  iff x=y. But here we only have d(x,x)=0
      double DistanceNeighborhoodNeighborhood(const Configuration *q_from, const Configuration *q_to);
      //Note: this is a pseudometric: invalidates second axiom of metric : d(x,y) = 0  iff x=y. But here we only have d(x,x)=0
      double DistanceConfigurationNeighborhood(const Configuration *q_from, const Configuration *q_to);

      double DistanceConfigurationConfigurationCover(const Configuration *q_from, const Configuration *q_to);

      std::vector<const Configuration*> GetInterpolationPath(const Configuration *q_from, const Configuration *q_to);
      std::vector<Vertex> GetCoverPath(const Vertex& v_source, const Vertex& v_sink);
      std::vector<const Configuration*> GetCoverPath(const Configuration *q_source, const Configuration *q_sink);

      //#######################################################################
      //Sampling Sample{Structure}{Substructure}
      //#######################################################################
      virtual Configuration* Sample();

      virtual Configuration* SampleCoverBoundary(std::string type);
      virtual Configuration* SampleCoverBoundary();
      Configuration* SampleCoverBoundaryValid(ob::PlannerTerminationCondition &ptc);
      void SampleRandomNeighborhoodBoundary(Configuration *q);
      bool SampleNeighborhoodBoundary(Configuration*, const Configuration*);
      bool SampleNeighborhoodBoundaryHalfBall(Configuration*, const Configuration*);

      void SampleGoal(Configuration*);
      void SampleUniform(Configuration*);
      virtual Configuration* SampleUniformQuotientCover(ob::State *state);
      //#######################################################################
      //Connect strategies
      //#######################################################################
      void Connect(const Configuration*, const Configuration*, Configuration*);
      //void Connect(const Configuration*, const Configuration*);
      void Grow(double t) override;
      bool GetSolution(ob::PathPtr &solution) override;

      virtual double GetImportance() const override;

      bool Interpolate(const Configuration*, Configuration*);
      bool Interpolate(const Configuration*, const Configuration*, Configuration*);
      bool Interpolate(const Configuration*, const Configuration*, double step_size, Configuration*);

      //#######################################################################
      //Neighborhood Set Computations
      //#######################################################################
      bool IsConfigurationInsideNeighborhood(Configuration *q, Configuration *qn);

      std::vector<Configuration*> GetConfigurationsInsideNeighborhood(Configuration *q);
      std::vector<Configuration*> GetConfigurationsIntersectingNeighborhood(Configuration *q);

      bool IsNeighborhoodInsideNeighborhood(Configuration *lhs, Configuration *rhs);
      void GetCosetFromQuotientSpace(Configuration *q);

      //#######################################################################
      //Cover Algorithms
      //#######################################################################

      Configuration* Nearest(Configuration *q) const;

      virtual void getPlannerData(base::PlannerData &data) const override;
      PlannerDataVertexAnnotated getAnnotatedVertex(Vertex vertex) const;
      PlannerDataVertexAnnotated getAnnotatedVertex(ob::State* state, double radius, bool sufficient) const;
      //#######################################################################
      RNG rng_;
      const double goalBias{0.1}; //in [0,1]
      const double voronoiBias{0.3}; //in [0,1]
      const double minimum_neighborhood_radius{1e-5}; //minimum allowed radius, otherwise configuration is considered INVALID 

      double totalVolumeOfCover{0.0};
      bool isConnected{false};
      bool saturated{false}; //if space is saturated, then we the whole free space has been found

      Graph graph;
      NearestNeighborsPtr nearest_cover{nullptr};
      NearestNeighborsPtr nearest_vertex{nullptr};
      Configuration *q_start{nullptr};
      Configuration *q_goal{nullptr};
      Vertex v_start;
      Vertex v_goal;
      PDF pdf_necessary_configurations;
      PDF pdf_all_configurations;
      std::vector<Vertex> shortest_path_start_goal;
      std::vector<Vertex> shortest_path_start_goal_necessary_vertices;

    public:
      Configuration* GetStartConfiguration() const;
      Configuration* GetGoalConfiguration() const;
      const Graph& GetGraph() const;
      const PDF& GetPDFNecessaryConfigurations() const;
      const PDF& GetPDFAllConfigurations() const;

      const NearestNeighborsPtr& GetNearestNeighborsCover() const;
      const NearestNeighborsPtr& GetNearestNeighborsVertex() const;

      double GetGoalBias() const;
      virtual void Print(std::ostream& out) const override;
      void Print(const Configuration *q, bool stopOnError=true) const;

    };
  }
}


