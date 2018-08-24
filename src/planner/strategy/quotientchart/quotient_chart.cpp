#include "quotient_chart.h"
#include "common.h"
#include <ompl/geometric/planners/prm/ConnectionStrategy.h>
#include <boost/foreach.hpp>

using namespace og;
#define foreach BOOST_FOREACH

QuotientChart::QuotientChart(const ob::SpaceInformationPtr &si, og::Quotient *parent_)
  : BaseT(si, parent_)
{
}
bool QuotientChart::IsSaturated() const
{
  return false;
}
void QuotientChart::setup() 
{
  if(!local_chart) BaseT::setup();
}
void QuotientChart::Grow(double t)
{
  growRoadmap(ob::timedPlannerTerminationCondition(t), xstates[0]);
}
double QuotientChart::GetImportance() const
{
  return importance;
}
void QuotientChart::SetImportance(double importance_)
{
  importance = importance_;
}
uint QuotientChart::GetLevel() const
{
  return level;
}
void QuotientChart::SetLevel(uint level_)
{
  level = level_;
}
uint QuotientChart::GetHorizontalIndex() const
{
  return horizontal_index;
}
void QuotientChart::SetHorizontalIndex(uint horizontal_index_)
{
  horizontal_index = horizontal_index_;
}

bool QuotientChart::FoundNewPath()
{
  ob::PathPtr sol;
  if(!hasSolution){
    CheckForSolution(sol);
    if(hasSolution) number_of_paths++;
    return hasSolution;
  }else{
    return false;
  }
}

void QuotientChart::AddSibling(QuotientChart *sibling_)
{
  siblings.push_back(sibling_);
}

void QuotientChart::SetSubGraph( QuotientChart *sibling, uint k )
{
  const og::QuotientGraph::Graph& Gprime = sibling->GetGraph();

  std::cout << std::string(80, '-') << std::endl;
  std::cout << "sibling graph: " << boost::num_vertices(Gprime) << " vertices | " << boost::num_edges(Gprime) << " edges."<< std::endl;

  //nn_ = sibling->GetRoadmapNeighborsPtr();

  local_chart = true;
  opt_ = sibling->opt_;
  level = sibling->GetLevel();
  startM_ = sibling->startM_;
  goalM_ = sibling->goalM_;
  number_of_paths = 0;

  enum CopyMode{ALL, CONNECTED_COMPONENT, SHORTEST_PATH, SHORTEST_PATH_LINEAR_HOMOTOPY};

  CopyMode mode = ALL;

  std::map<Vertex, Vertex> GprimetoG;
  switch(mode){
    case ALL:
      {
        //###########################################################################
        //Copy the whole graph
        //###########################################################################
        boost::copy_graph( Gprime, G);
        break;
      }
    case CONNECTED_COMPONENT:
      {
        //###########################################################################
        //Copy connected component of the start node
        //###########################################################################
        foreach (Vertex vprime, boost::vertices(Gprime))
        {
          if(sibling->sameComponent(vprime, startM_.at(0))){
            Vertex v1 = boost::add_vertex(G);
            G[v1] = Gprime[vprime];
            GprimetoG[vprime] = v1;
          }
        }
        foreach (Edge eprime, boost::edges(Gprime))
        {
          const Vertex v1 = boost::source(eprime, Gprime);
          const Vertex v2 = boost::target(eprime, Gprime);
          if(sibling->sameComponent(v1, startM_.at(0))){
            boost::add_edge(GprimetoG[v1], GprimetoG[v2], Gprime[eprime], G);
          }
        }
        break;
      }
    case SHORTEST_PATH:
      {
        //###########################################################################
        //Copy only the edges and vertices along the shortest path
        //###########################################################################
        startM_.clear();
        goalM_.clear();
        std::vector<Vertex> shortestVertexPath = sibling->shortestVertexPath_;

        Vertex v1 = boost::add_vertex(G);
        for(uint k = 0; k < shortestVertexPath.size()-1; k++){
          Vertex v1prime = shortestVertexPath.at(k);
          Vertex v2prime = shortestVertexPath.at(k+1);

          Vertex v2 = boost::add_vertex(G);
          G[v1] = Gprime[v1prime];
          G[v2] = Gprime[v2prime];

          // double d1 = Gprime[v1prime].open_neighborhood_distance;
          // double d2 = G[v1].open_neighborhood_distance;
          // std::cout << d1 << "->" << d2 << std::endl;

          Edge eprime = boost::edge(v1prime,v2prime,Gprime).first;

          if(k==0) startM_.push_back(v1);
          if(k==shortestVertexPath.size()-2) goalM_.push_back(v2);
          boost::add_edge(v1, v2, Gprime[eprime], G);

          v1 = v2;
        }
        break;
      }
    case SHORTEST_PATH_LINEAR_HOMOTOPY:
      {
        //###########################################################################
        //Copy edges and vertices along the shortest path
        //+ all edges belonging to same path space cover
        //###########################################################################
        startM_.clear();
        goalM_.clear();
      }
    default:
      {
        std::cout << "Mode " << mode << " not recognized." << std::endl;
        exit(0);
      }
  }
  std::cout << "new graph: " << GetNumberOfVertices() << " vertices | " << GetNumberOfEdges() << " edges."<< std::endl;
}

uint QuotientChart::GetNumberOfPaths() const
{
  return number_of_paths;
}

uint QuotientChart::GetNumberOfSiblings() const
{
  return siblings.size();
}

PlannerDataVertexAnnotated QuotientChart::getPlannerDataVertex(ob::State *state, Vertex v) const
{
  PlannerDataVertexAnnotated p(state);
  p.SetOpenNeighborhoodDistance(G[v].open_neighborhood_distance);
  return p;
}

PlannerDataVertexAnnotated QuotientChart::getAnnotatedVertex(Vertex vertex, std::vector<int> &path, std::map<const Vertex, ob::State*> &vertexToStates) const
{
  ob::State *state = (local_chart?si_->cloneState(G[vertex].state):G[vertex].state);
  vertexToStates[vertex] = state;
  PlannerDataVertexAnnotated pvertex = getPlannerDataVertex(state, vertex);
  pvertex.SetLevel(level);
  pvertex.SetPath(path);
  return pvertex;
}

void QuotientChart::getPlannerData(ob::PlannerData &data) const
{
  uint Nvertices = data.numVertices();
  uint Nedges = data.numEdges();

  //###########################################################################
  //Get Path for this chart
  //###########################################################################
  std::vector<int> path;

  path.push_back(GetHorizontalIndex());
  og::QuotientChart *parent = static_cast<og::QuotientChart*>(GetParent());
  while(parent!=nullptr)
  {
    path.push_back(parent->GetHorizontalIndex());
    parent = static_cast<og::QuotientChart*>(parent->GetParent());
  }

  std::reverse(std::begin(path), std::end(path));

  //###########################################################################
  //Get Data from this chart
  //###########################################################################

  //if the chart is local, we need to clone new states such that we have
  //duplicate vertices (charts can overlap). 

  std::map<const Vertex, ob::State*> vertexToStates;

  for (Vertex vs : startM_)
  {
    PlannerDataVertexAnnotated pstart = getAnnotatedVertex(vs, path, vertexToStates);
    data.addStartVertex(pstart);
  }

  for (Vertex vg : goalM_)
  {
    PlannerDataVertexAnnotated pgoal = getAnnotatedVertex(vg, path, vertexToStates);
    data.addGoalVertex(pgoal);
  }

  foreach( const Vertex v, boost::vertices(G))
  {
    if(vertexToStates.find(v) == vertexToStates.end()) {
      PlannerDataVertexAnnotated p = getAnnotatedVertex(v, path, vertexToStates);
      data.addVertex(p);
    }
    //otherwise vertex is a goal or start vertex and has already been added
  }
  foreach (const Edge e, boost::edges(G))
  {
    const Vertex v1 = boost::source(e, G);
    const Vertex v2 = boost::target(e, G);

    ob::State *s1 = vertexToStates[v1];
    ob::State *s2 = vertexToStates[v2];
    PlannerDataVertexAnnotated p1(s1); //creates same vertex!
    PlannerDataVertexAnnotated p2(s2);
    data.addEdge(p1,p2);
  }

  //###########################################################################
  //Get Data From all siblings
  //###########################################################################

  std::cout << "[QuotientChart] vIdx " << level << " | hIdx " << horizontal_index 
    << " | siblings " << siblings.size() << " | path " << path 
    << " | vertices " << data.numVertices() - Nvertices 
    << " | edges " << data.numEdges() - Nedges
    << std::endl;

  for(uint i = 0; i < siblings.size(); i++){
    siblings.at(i)->getPlannerData(data);
  }
  if(child!=nullptr) child->getPlannerData(data);
}
