#include "open_set_hypersphere.h"
#include <ompl/base/State.h>
#include <ompl/base/SpaceInformation.h>

using namespace cover;

OpenSetHypersphere::OpenSetHypersphere(ob::SpaceInformationPtr si_, ob::State *s, double dist_robot_env_):
  OpenSet(s), si(si_), radius(dist_robot_env_)
{
}

double OpenSetHypersphere::Distance(const ob::State *s_lhs, const ob::State *s_rhs){
  return si->distance(s_lhs, s_rhs);
}
bool OpenSetHypersphere::IsInside(ob::State *sPrime)
{
  return Distance(sCenter, sPrime) < radius;
}
double OpenSetHypersphere::GetRadius()
{
  return radius;
}

void OpenSetHypersphere::DrawGL(GUIState&)
{
  if(state("draw_roadmap_volume")){
    glDisable(GL_LIGHTING);
    glEnable(GL_BLEND);
    drawPoint(center);
    glTranslate(center);
    setColor(cOpenSet);
    drawSphere(radius,16,8);
    glDisable(GL_BLEND);
    glEnable(GL_LIGHTING);
  }
}
