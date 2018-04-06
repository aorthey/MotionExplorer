#include "open_set_hypersphere.h"
#include "gui/drawMotionPlanner.h"
#include "gui/colors.h"
#include <ompl/base/State.h>
#include <ompl/base/SpaceInformation.h>

using namespace cover;

OpenSetHypersphere::OpenSetHypersphere(CSpaceOMPL *cspace_, ob::State *s, double dist_robot_env_):
  OpenSet(cspace_,s), radius(dist_robot_env_)
{
}

double OpenSetHypersphere::Distance(const ob::State *s_lhs, const ob::State *s_rhs){
  return cspace->SpaceInformationPtr()->distance(s_lhs, s_rhs);
}
bool OpenSetHypersphere::IsInside(ob::State *sPrime)
{
  return Distance(sCenter, sPrime) < radius;
}
double OpenSetHypersphere::GetRadius() const
{
  return radius;
}
bool OpenSetHypersphere::IsSubsetOf(const cover::OpenSet *rhs_, double tolerance) const
{
  const cover::OpenSetHypersphere *rhs = static_cast<const cover::OpenSetHypersphere*>(rhs_);
  return (fabs(radius - rhs->GetRadius())<tolerance);
}

void OpenSetHypersphere::DrawGL(GUIState& state)
{
  if(state("draw_roadmap_volume")){
    glDisable(GL_LIGHTING);
    glEnable(GL_BLEND);
    Vector3 q = cspace->getXYZ(sCenter);
    drawPoint(q);
    glTranslate(q);
    setColor(cOpenSet);
    drawSphere(radius,16,8);
    glDisable(GL_BLEND);
    glEnable(GL_LIGHTING);
  }
}
