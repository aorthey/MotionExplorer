#pragma once

class SimplicialComplex
{
  //implements vietoris rips complex
  class VertexInternalState{
    public:
      VertexInternalState() = default;
      VertexInternalState(const VertexInternalState &vis) = default;
      ob::State *state{nullptr};
      double open_neighborhood_distance{0};
  };

  class EdgeInternalState{
    public:
      EdgeInternalState() = default;
      EdgeInternalState(double weight_): weight(weight_){};
      void setWeight(double weight_){
        weight = weight_;
      }
      double getWeight(){
        return weight;
      }
    private:
      double weight;
  };

  typedef boost::adjacency_list<
     boost::vecS, 
     boost::vecS, 
     boost::undirectedS,
     VertexInternalState,
     EdgeInternalState
   > Graph;

  typedef boost::graph_traits<Graph> BGT;
  typedef boost::graph_traits<Graph>::vertex_descriptor Vertex;
  typedef boost::graph_traits<Graph>::edge_descriptor Edge;
  typedef boost::graph_traits<Graph>::vertices_size_type VertexIndex;
  typedef boost::graph_traits<Graph>::in_edge_iterator IEIterator;
  typedef boost::graph_traits<Graph>::out_edge_iterator OEIterator;
  typedef Vertex* VertexParent;
  typedef VertexIndex* VertexRank;
  typedef std::shared_ptr<NearestNeighbors<Vertex>> RoadmapNeighbors;
  typedef std::function<const std::vector<Vertex> &(const Vertex)> ConnectionStrategy;

  public:
    RoadmapNeighbors nn_infeasible;
    RoadmapNeighbors nn_feasible;

    SimplicialComplex

    void AddVertex(ob::State *s);
    void AddEdge(ob::State *s1, ob::State *s2);

    void RemoveEdge(Vertex v1, Vertex v2);


}

