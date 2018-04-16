#include "nef_polyhedron.h"
#include "gui/common.h"
#include "klampt.h"
#include "planner/cspace/cspace.h"
#include <CGAL/convex_decomposition_3.h> 
#include <CGAL/OFF_to_nef_3.h>
#include <CGAL/convex_hull_3.h>
#include <CGAL/Convex_hull_3/dual/halfspace_intersection_3.h>

typedef Nef_polyhedron_3::Halfedge                                  Nef_halfedge;
typedef Nef_polyhedron_3::Vertex_const_iterator                     Nef_vertex_const_iterator;
typedef Nef_polyhedron_3::Halfedge_const_iterator                   Nef_halfedge_const_iterator;
typedef Nef_polyhedron_3::Halffacet_const_iterator                  Nef_halffacet_const_iterator;
typedef Nef_polyhedron_3::Halffacet_cycle_const_iterator            Nef_halffacet_cycle_const_iterator;
typedef Nef_polyhedron_3::Volume_const_iterator                     Nef_volume_const_iterator;

using namespace GLDraw;

NefPolyhedron::NefPolyhedron( const ConvexPolyhedron* p_)
{
  poly = new Nef_polyhedron_3(p_->GetCGALPolyhedronNonConst());
}

// poly = poly - union(O_1,...,O_k)
void NefPolyhedron::SubtractObstacles(CSpaceOMPL *cspace)
{
  RobotWorld* world = cspace->GetWorldPtr();
  for(uint k = 0; k < world->terrains.size(); k++)
  {
    SmartPointer<Terrain> terrain = world->terrains.at(k);
    std::string f = terrain->geomFile;
    f = f.substr(0, f.length()-3)+"off";
    std::cout << f << std::endl;

    std::ifstream off_file(f.c_str());
    //assert(off_file.good());
    if(!off_file.good()){
      std::cout << "could not load file " << f << std::endl;
      exit(1);
    }

    std::cout << "X = load(*.off) ..." << std::endl;
    Nef_polyhedron_3 O_k;
    std::size_t discarded = CGAL::OFF_to_nef_3(off_file, O_k);
    if(discarded>0) std::cout << "WARNING: discarded " << discarded << "/" << O_k.number_of_facets() << " triangle elements from file " << f << std::endl;
    std::cout << "X = X - obstacles ..." << std::endl;
    *poly -= O_k;
  }
  std::cout << "done" << std::endl;


}
std::vector<ConvexPolyhedron> NefPolyhedron::GetConvexDecomposition()
{
  CGAL::convex_decomposition_3(*poly);

  std::vector<ConvexPolyhedron> parts;

  // the first volume is the outer volume, which is 
  // ignored in the decomposition
  Nef_volume_const_iterator ci = ++poly->volumes_begin();
  for( ; ci != poly->volumes_end(); ++ci) {
    if(ci->mark()) {
      Polyhedron_3 P;
      poly->convert_inner_shell_to_polyhedron(ci->shells_begin(), P);
      ConvexPolyhedron cp(P);
      parts.push_back(cp);
    }
  }
  //std::cout << "decomposition into " << poly->convex_parts.size() << " convex parts " << std::endl;
  return parts;
}

void NefPolyhedron::DrawGL(GUIState& state)
{
  if(state("draw_cover_vertices")){
    glPointSize(10);
    setColor(black);
    for ( Nef_vertex_const_iterator v = poly->vertices_begin(); v != poly->vertices_end(); ++v)
    {
      Point_3 p = v->point();
      double x = CGAL::to_double(p[0]);
      double y = CGAL::to_double(p[1]);
      double z = CGAL::to_double(p[2]);
      Vector3 q(x,y,z);
      drawPoint(q);
    }
  }
  if(state("draw_cover_edges")){
    glLineWidth(3);
    setColor(black);
    for ( Nef_halfedge_const_iterator e = poly->halfedges_begin(); e != poly->halfedges_end(); ++e)
    {
      Point_3 v0 = e->source()->point();
      Point_3 v1 = e->target()->point();
      //Vector3 q0(v0[0],v0[1],v0[2]);
      double x0 = CGAL::to_double(v0[0]);
      double y0 = CGAL::to_double(v0[1]);
      double z0 = CGAL::to_double(v0[2]);
      double x1 = CGAL::to_double(v1[0]);
      double y1 = CGAL::to_double(v1[1]);
      double z1 = CGAL::to_double(v1[2]);
      Vector3 q0(x0,y0,z0);
      Vector3 q1(x1,y1,z1);
      drawPoint(q0);
      drawLineSegment(q0,q1);
    }
  }
  if(state("draw_cover_faces")){
    setColor(magenta);
    for ( Nef_halffacet_const_iterator f = poly->halffacets_begin(); f != poly->halffacets_end(); ++f)
    {
      Nef_halffacet_cycle_const_iterator fcirc = f->facet_cycles_begin();

      glBegin(GL_POLYGON);
      CGAL_For_all(fcirc, f->facet_cycles_end()) 
      {
        Nef_polyhedron_3::SHalfedge_const_handle se = Nef_polyhedron_3::SHalfedge_const_handle(fcirc);
        Nef_polyhedron_3::SHalfedge_around_facet_const_circulator hc_start(se);
        Nef_polyhedron_3::SHalfedge_around_facet_const_circulator hc_end(hc_start);
        CGAL_For_all(hc_start, hc_end)
        {
          glVertex3f(CGAL::to_double(hc_start->source()->center_vertex()->point().x()),
                     CGAL::to_double(hc_start->source()->center_vertex()->point().y()),
                     CGAL::to_double(hc_start->source()->center_vertex()->point().z()));
        }
      }
      glEnd();
    }
  }

}
