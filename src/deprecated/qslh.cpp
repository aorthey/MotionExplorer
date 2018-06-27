#include "qslh.h"
#include "planner/cover/open_set_convex.h"
#include "elements/plannerdata_vertex_annotated.h"
#include "planner/validitychecker/validity_checker_simplicial_complex.h"
#include <ompl/util/Console.h>

using namespace og;
namespace ompl
{
  namespace magic
  {
    static const double LH_DELTA = 0.1;
  }
}
QSLH::QSLH(const ob::SpaceInformationPtr &si, Quotient *previous_):
  Base(si, previous_)
{
  openNeighborhood_ = (boost::get(vertex_open_neighborhood_t(), g_));
  setName("QSLH"+std::to_string(id));
}

QSLH::~QSLH()
{
}

void QSLH::Init()
{

  Base::Init();

  //if(previous==nullptr)
  //{
  //  const ob::State *start = stateProperty_[startM_.at(0)];
  //  const ob::State *goal  = stateProperty_[goalM_.at(0)];
  //  auto checkerPtr = static_pointer_cast<ValidityCheckerSimplicialComplex>(si_->getStateValidityChecker());

  //}
}

bool QSLH::SampleGraph(ob::State *q_random_graph)
{
  return true;
}

void QSLH::Grow(double t)
{
  if(previous==nullptr)
  {
    Base::Grow(t);
  }else{
    return;
  }
}
void QSLH::CheckForSolution(ob::PathPtr &solution)
{
  if(previous!=nullptr){
    hasSolution = true;
    return;
  }
  Base::CheckForSolution(solution);
  if(hasSolution)
  {
    //compute linear homotopy class of shortest path $p$. 
    //This is defined by subgraph $L(p) \subset G$.

    //std::vector<Vertex> startM_;
    //std::vector<Vertex> goalM_;
    //std::vector<Vertex> shortestVertexPath_;
    //for(uint k = 0; k < shortestVertexPath_.size(); k++){
    //  const Vertex &vm = shortestVertexPath_.at(k);
    //  const std::vector<Vertex> &neighbors = connectionStrategy_(vm);
    //  for(uint j = 0; j < neighbors.size(); j++){
    //    const Vertex &vn = neighbors.at(j)
    //    if(Distance(vm, vn) < LH_DELTA)
    //    {
    //    }
    //  }
    //}

    std::cout << "found solution" << std::endl;
  }

}

void QSLH::getPlannerData(ob::PlannerData &data) const
{
  Base::getPlannerData(data);

  uint N = data.numVertices();
  std::cout << "added " << data.numVertices()-N << " vertices." << std::endl;

  for(uint vidx = N; vidx < data.numVertices(); vidx++){
    PlannerDataVertexAnnotated *v = static_cast<PlannerDataVertexAnnotated*>(&data.getVertex(vidx));
    v->SetMaxPathClass(2);
    v->SetPathClass(0);
  }
}
