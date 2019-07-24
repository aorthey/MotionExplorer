#pragma once
#include "planner/strategy/quotientgraph/quotient_graph.h"

namespace og = ompl::geometric;
class PathEnumerator
{
public:
  PathEnumerator(og::QuotientGraph::Vertex v_start, og::QuotientGraph::Vertex v_goal, og::QuotientGraph::Graph &graph):
    v_start_(v_start), v_goal_(v_goal), graph_(graph)
  {
      numVertices = boost::num_vertices(graph);
      for(uint k = 0; k < numVertices; k++){
          visited.push_back(false);
      }
  };

  void ComputePaths(){
      ComputePathsRecurse(v_start_, v_goal_);
  }
  void ComputePathsRecurse(og::QuotientGraph::Vertex u, og::QuotientGraph::Vertex d)
  {
      visited.at(u) = true; 
      path.push_back(u);
      path_index++;

      if(u == d){
          std::vector<og::QuotientGraph::Vertex> vpath;
          for (int i = 0; i<path_index; i++) 
          {
              vpath.push_back(i);
              std::cout << path.at(i) << " "; 
          }
          std::cout << std::endl; 
          pathStack.push_back(vpath);
      }else{
        og::QuotientGraph::OEIterator ei, ei_end;
          for (boost::tie(ei, ei_end) = boost::out_edges(u, graph_); ei != ei_end; ++ei) {
            og::QuotientGraph::Vertex v_source = boost::source ( *ei, graph_ );
            og::QuotientGraph::Vertex v_target = boost::target ( *ei, graph_ );
            assert(v_source == u);

            if (!visited[v_target]){
                ComputePathsRecurse(v_target, d);
            }
          }

      }
      path_index--;
      visited.at(u) = false; 
  }

  protected:
    int path_index{0};
    unsigned numVertices{0};
    std::vector<bool> visited;
    std::vector<og::QuotientGraph::Vertex> path;
    std::vector<std::vector<og::QuotientGraph::Vertex>> pathStack;
    og::QuotientGraph::Vertex v_start_;
    og::QuotientGraph::Vertex v_goal_;
    og::QuotientGraph::Graph &graph_;

};
