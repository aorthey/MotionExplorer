//
//
////A cover of a quotient-space
//template<typename T>
//class QuotientCover{
//
//  class VertexInternalState{
//    public:
//      VertexInternalState() = default;
//      VertexInternalState(const VertexInternalState &vis)
//      ob::State *state{nullptr};
//      VertexInternalState *coset{nullptr};
//      T* internals;
//  };
//
//  class EdgeInternalState{
//    public:
//      EdgeInternalState() = default;
//      EdgeInternalState(double weight_): weight(weight_);
//      EdgeInternalState(const EdgeInternalState &edge_rhs)
//      {
//        weight = edge_rhs.GetWeight();
//      }
//      void SetWeight(double d){
//        cost = ob::Cost(d);
//      }
//      ob::Cost GetCost(){
//        return ob::Cost(weight);
//      }
//      double GetWeight(){
//        return weight;
//      }
//    private:
//      double weight{+dInf};
//  };
//
//  typedef boost::adjacency_list<
//     boost::vecS, 
//     boost::vecS, 
//     boost::undirectedS,
//     VertexInternalState,
//     EdgeInternalState
//   > Graph;
//
//  typedef boost::graph_traits<Graph> GT;
//  typedef GT::vertex_descriptor Vertex;
//  typedef GT::edge_descriptor Edge;
//  typedef GT::vertices_size_type VertexIndex;
//  typedef GT::in_edge_iterator IEIterator;
//  typedef GT::out_edge_iterator OEIterator;
//}
