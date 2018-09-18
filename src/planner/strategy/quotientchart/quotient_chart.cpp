#include "quotient_chart.h"
#include "common.h"
#include <ompl/geometric/planners/prm/ConnectionStrategy.h>
#include <boost/foreach.hpp>

using namespace og;
#define foreach BOOST_FOREACH

QuotientChart::QuotientChart(const ob::SpaceInformationPtr &si, og::Quotient *parent_)
  : BaseT(si, parent_)
{
  UpdateChartPath();
}

void QuotientChart::setup() 
{
  if(!isLocalChart) BaseT::setup();
}
uint QuotientChart::GetChartHorizontalIndex() const
{
  return chartHorizontalIndex;
}
void QuotientChart::SetChartHorizontalIndex(uint chartHorizontalIndex_)
{
  chartHorizontalIndex = chartHorizontalIndex_;

  UpdateChartPath();
}

void QuotientChart::UpdateChartPath() 
{
  chartPath.clear();
  chartPath.push_back(chartHorizontalIndex);
  og::QuotientChart *parent = static_cast<og::QuotientChart*>(GetParent());
  while(parent!=nullptr)
  {
    chartPath.push_back(parent->GetChartHorizontalIndex());
    parent = static_cast<og::QuotientChart*>(parent->GetParent());
  }
  std::reverse(std::begin(chartPath), std::end(chartPath));
}

bool QuotientChart::FoundNewPath()
{
  ob::PathPtr sol;
  if(!hasSolution){
    hasSolution = GetSolution(sol);
    if(hasSolution) chartNumberOfComponents++;
    return hasSolution;
  }else{
    return false;
  }
}

void QuotientChart::AddChartSibling(QuotientChart *sibling_)
{
  chartSiblings.push_back(sibling_);
}

// void QuotientChart::CopyChartFromSibling( QuotientChart *sibling, uint k )
// {
//   const og::QuotientGraph::Graph& Gprime = sibling->GetGraph();

//   std::cout << std::string(80, '-') << std::endl;
//   std::cout << "sibling graph: " << boost::num_vertices(Gprime) << " vertices | " << boost::num_edges(Gprime) << " edges."<< std::endl;

//   //nn_ = sibling->GetRoadmapNeighborsPtr();

//   local_chart = true;
//   opt_ = sibling->opt_;
//   level = sibling->GetLevel();
//   startM_ = sibling->startM_;
//   goalM_ = sibling->goalM_;
//   number_of_paths = 0;

//   enum CopyMode{ALL, CONNECTED_COMPONENT, SHORTEST_PATH, SHORTEST_PATH_LINEAR_HOMOTOPY};

//   CopyMode mode = ALL;

//   std::map<Vertex, Vertex> GprimetoG;
//   switch(mode){
//     case ALL:
//       {
//         //###########################################################################
//         //Copy the whole graph
//         //###########################################################################
//         boost::copy_graph( Gprime, G);
//         break;
//       }
//     case CONNECTED_COMPONENT:
//       {
//         //###########################################################################
//         //Copy connected component of the start node
//         //###########################################################################
//         foreach (Vertex vprime, boost::vertices(Gprime))
//         {
//           if(sibling->sameComponent(vprime, startM_.at(0))){
//             Vertex v1 = boost::add_vertex(G);
//             G[v1] = Gprime[vprime];
//             GprimetoG[vprime] = v1;
//           }
//         }
//         foreach (Edge eprime, boost::edges(Gprime))
//         {
//           const Vertex v1 = boost::source(eprime, Gprime);
//           const Vertex v2 = boost::target(eprime, Gprime);
//           if(sibling->sameComponent(v1, startM_.at(0))){
//             boost::add_edge(GprimetoG[v1], GprimetoG[v2], Gprime[eprime], G);
//           }
//         }
//         break;
//       }
//     case SHORTEST_PATH:
//       {
//         //###########################################################################
//         //Copy only the edges and vertices along the shortest path
//         //###########################################################################
//         startM_.clear();
//         goalM_.clear();
//         std::vector<Vertex> shortestVertexPath = sibling->shortestVertexPath_;

//         Vertex v1 = boost::add_vertex(G);
//         for(uint k = 0; k < shortestVertexPath.size()-1; k++){
//           Vertex v1prime = shortestVertexPath.at(k);
//           Vertex v2prime = shortestVertexPath.at(k+1);

//           Vertex v2 = boost::add_vertex(G);
//           G[v1] = Gprime[v1prime];
//           G[v2] = Gprime[v2prime];

//           // double d1 = Gprime[v1prime].open_neighborhood_distance;
//           // double d2 = G[v1].open_neighborhood_distance;
//           // std::cout << d1 << "->" << d2 << std::endl;

//           Edge eprime = boost::edge(v1prime,v2prime,Gprime).first;

//           if(k==0) startM_.push_back(v1);
//           if(k==shortestVertexPath.size()-2) goalM_.push_back(v2);
//           boost::add_edge(v1, v2, Gprime[eprime], G);

//           v1 = v2;
//         }
//         break;
//       }
//     case SHORTEST_PATH_LINEAR_HOMOTOPY:
//       {
//         //###########################################################################
//         //Copy edges and vertices along the shortest path
//         //+ all edges belonging to same path space cover
//         //###########################################################################
//         startM_.clear();
//         goalM_.clear();
//       }
//     default:
//       {
//         std::cout << "Mode " << mode << " not recognized." << std::endl;
//         exit(0);
//       }
//   }
//   std::cout << "new graph: " << GetNumberOfVertices() << " vertices | " << GetNumberOfEdges() << " edges."<< std::endl;
// }

uint QuotientChart::GetChartNumberOfComponents() const
{
  return chartNumberOfComponents;
}

uint QuotientChart::GetChartNumberOfSiblings() const
{
  return chartSiblings.size();
}
std::vector<int> QuotientChart::GetChartPath() const
{
  return chartPath;
}
bool QuotientChart::IsSaturated() const
{
  return false;
}

// PlannerDataVertexAnnotated QuotientChart::getPlannerDataVertex(ob::State *state, Vertex v) const
// {
//   PlannerDataVertexAnnotated p(state);
//   p.SetOpenNeighborhoodDistance(G[v].open_neighborhood_distance);
//   return p;
// }

// PlannerDataVertexAnnotated QuotientChart::getAnnotatedVertex(Vertex vertex, std::vector<int> &path, std::map<const Vertex, ob::State*> &vertexToStates) const
// {
//   ob::State *state = (local_chart?si_->cloneState(G[vertex].state):G[vertex].state);
//   vertexToStates[vertex] = state;
//   PlannerDataVertexAnnotated pvertex = getPlannerDataVertex(state, vertex);
//   pvertex.SetLevel(level);
//   pvertex.SetPath(path);
//   return pvertex;
// }

void QuotientChart::getPlannerData(ob::PlannerData &data) const
{
  uint Nvertices = data.numVertices();
  uint Nedges = data.numEdges();

  //###########################################################################
  //Get Data from this chart
  //###########################################################################
  //if the chart is local, we need to clone new states such that we have
  //duplicate vertices (charts can overlap). 

  // std::map<const Vertex, ob::State*> vertexToStates;

  // for (Vertex vs : startM_)
  // {
  //   PlannerDataVertexAnnotated pstart = getAnnotatedVertex(vs, chart_path, vertexToStates);
  //   data.addStartVertex(pstart);
  // }

  // for (Vertex vg : goalM_)
  // {
  //   PlannerDataVertexAnnotated pgoal = getAnnotatedVertex(vg, chart_path, vertexToStates);
  //   data.addGoalVertex(pgoal);
  // }

  // foreach( const Vertex v, boost::vertices(G))
  // {
  //   if(vertexToStates.find(v) == vertexToStates.end()) {
  //     PlannerDataVertexAnnotated p = getAnnotatedVertex(v, chart_path, vertexToStates);
  //     data.addVertex(p);
  //   }
  //   //otherwise vertex is a goal or start vertex and has already been added
  // }
  // foreach (const Edge e, boost::edges(G))
  // {
  //   const Vertex v1 = boost::source(e, G);
  //   const Vertex v2 = boost::target(e, G);

  //   ob::State *s1 = vertexToStates[v1];
  //   ob::State *s2 = vertexToStates[v2];
  //   PlannerDataVertexAnnotated p1(s1); //creates same vertex!
  //   PlannerDataVertexAnnotated p2(s2);
  //   data.addEdge(p1,p2);
  // }
  getPlannerDataAnnotated(data);

  //###########################################################################
  //Get Data From all siblings
  //###########################################################################

  std::cout << "[QuotientChart" << id << "] " << (isLocalChart?"[local]":"") << " level " << level << " | hIdx " << chartHorizontalIndex
    << " | siblings " << chartSiblings.size() << " | chart_path " << chartPath
    << " | vertices " << data.numVertices() - Nvertices 
    << " | edges " << data.numEdges() - Nedges
    << std::endl;

  for(uint i = 0; i < chartSiblings.size(); i++){
    chartSiblings.at(i)->getPlannerData(data);
  }
  if(child!=nullptr) child->getPlannerData(data);
}

void QuotientChart::Print(std::ostream& out) const
{
  BaseT::Print(out);
  out << std::endl << "  [Chart] " << (isLocalChart?"(local)":"") << " level " << level << " | hIdx " << chartHorizontalIndex
    << " | siblings " << chartSiblings.size() << " | chart_path " << chartPath;
}
