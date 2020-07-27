#include "ViewLocalMinimaTree.h"
#include "elements/path_pwl.h"
#include <ompl/base/State.h>
using namespace Math3D;
using namespace ompl::multilevel;
using namespace ompl::base;

ViewLocalMinimaTree::ViewLocalMinimaTree(ompl::multilevel::LocalMinimaTreePtr localMinimaTree,
   std::vector<CSpaceOMPL*> cspace_levels):
  x_(0),
  y_(0),
  heightPerLevel(100),
  width(400),
  width_column1(0),
  width_column2(0),
  width_column3(width-width_column1-width_column2),
  drawBound(true),
  drawPlotArea(true), 
  localMinimaTree_(localMinimaTree),
  cspace_levels_(cspace_levels)
{
  textColor.set(0.1,0.1,0.1);
  nodeColor.set(0.8,0.8,0.8);
  nodeSelectedColor.set(0.4,0.4,0.4);
  nodeUnselectedColor.set(0.8,0.8,0.8);
  boundColor.set(0.2,0.2,0.2);
  plotAreaColor.set(0.8,0.8,0.8,0.5);
  node_radius = heightPerLevel/4.0;
  node_radius_unselected = node_radius - node_radius/4;
  node_radius_selected = node_radius;
}

PathPiecewiseLinear* ViewLocalMinimaTree::getPathSelected()
{
  return pathSelected_;
}

void ViewLocalMinimaTree::DrawGLNodeSelected(GUIState& state, LocalMinimaNode* node)
{
  if(node == nullptr) return;

  int level = node->getLevel();
  if(node->customRepresentation == nullptr)
  {
      auto path = node->asPathPtr();
      PathPiecewiseLinear *pwl = new PathPiecewiseLinear(path, cspace_levels_.back(),
          cspace_levels_.at(level));
      node->customRepresentation = pwl;
  }

  pathSelected_ = static_cast<PathPiecewiseLinear*>(node->customRepresentation);


  pathSelected_->zOffset = 0.001;
  pathSelected_->ptsize = 1;
  pathSelected_->linewidth = 0.05;
  pathSelected_->widthBorder = 0.001;
  pathSelected_->setColor(magenta);
  pathSelected_->drawSweptVolume = false;
  pathSelected_->drawCross = false;
  pathSelected_->DrawGL(state);

}
void ViewLocalMinimaTree::DrawGLNodeUnSelected(GUIState& state, LocalMinimaNode* node)
{
  if(node == nullptr) return;

  if(node->customRepresentation == nullptr)
  {
      auto path = node->asPathPtr();
      PathPiecewiseLinear *pwl = new PathPiecewiseLinear(path, cspace_levels_.back(),
          cspace_levels_.at(node->getLevel()));
      node->customRepresentation = pwl;
  }

  PathPiecewiseLinear *pwl = static_cast<PathPiecewiseLinear*>(node->customRepresentation);

  pwl->zOffset = 0.001;
  pwl->ptsize = 1;
  pwl->linewidth = 0.03;
  pwl->widthBorder = 0.001;
  pwl->setColor(grey);
  pwl->drawSweptVolume = false;
  pwl->drawCross = false;
  pwl->DrawGL(state);

}
void ViewLocalMinimaTree::DrawGL(GUIState& state)
{
  std::lock_guard<std::recursive_mutex> guard(localMinimaTree_->getLock());

  LocalMinimaNode* node = localMinimaTree_->getSelectedPath();
  DrawGLNodeSelected(state, node);

  std::vector<LocalMinimaNode*> nodeVector = localMinimaTree_->getSelectedPathSiblings();
  for(uint k = 0; k < nodeVector.size(); k++)
  {
      DrawGLNodeUnSelected(state, nodeVector.at(k));
  }

}

void ViewLocalMinimaTree::DrawGLScreen(double x, double y)
{
  std::lock_guard<std::recursive_mutex> guard(localMinimaTree_->getLock());


  int lmtLevel = localMinimaTree_->getNumberOfLevelContainingMinima();
  std::vector<int> level_nodes;
  std::vector<int> selected_path = localMinimaTree_->getSelectedMinimum();

  for(int k = 0; k < lmtLevel; k++)
  {
    level_nodes.push_back(localMinimaTree_->getNumberOfMinima(k));
  }

  double xprev = x+width_column1+width_column2+1.5*node_radius;
  double yprev = y+0.5*heightPerLevel;
  double xstep = node_radius*3;
  double ystep = heightPerLevel;

  double height = ystep * (lmtLevel+1);

  int maxNodes = 1;
  int addNodes = 0;
  for(uint k = 0; k < level_nodes.size(); k++){
    int nk = level_nodes.at(k) + addNodes;
    if(nk > maxNodes) maxNodes = nk;
    if(k < selected_path.size())
      addNodes += selected_path.at(k);
  }

  width_column3 = maxNodes*xstep;
  double width = width_column1 + width_column2 + width_column3;

  glDisable(GL_LIGHTING);
  glDisable(GL_DEPTH_TEST);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);

  if(drawBound) {
    boundColor.setCurrentGL();
    glLineWidth(3);
    glBegin(GL_LINE_LOOP);
    glVertex2i(x,y);
    glVertex2i(x+width,y);
    glVertex2i(x+width,y+height);
    glVertex2i(x,y+height);
    glEnd();
  }
  if(drawPlotArea) {
    plotAreaColor.setCurrentGL();
    glBegin(GL_QUADS);
    glVertex2i(x,y);
    glVertex2i(x,y+height);
    glVertex2i(x+width,y+height);
    glVertex2i(x+width,y);
    glEnd();
  }


  nodeColor = nodeUnselectedColor;
  std::string emptySet; emptySet = "#";

  DrawNode(xprev, yprev, node_radius_selected, emptySet);

  for(uint k = 0; k < level_nodes.size(); k++){
    double yn = y+(k+1)*heightPerLevel+0.5*heightPerLevel;

    int nodes_on_level = level_nodes.at(k);
    int selected_node = -1;
    if(k < selected_path.size())
    {
      selected_node = selected_path.at(k);
    }

    for(int i = 0; i < nodes_on_level; i++){
      double xn = xprev + xstep*i;
      double r = (i==selected_node? node_radius_selected: node_radius_unselected);
      nodeColor = (i==selected_node? nodeSelectedColor:nodeUnselectedColor);
      DrawNode(xn,yn,r,i);
      (i==selected_node? glLineWidth(4): glLineWidth(2));
      DrawLineFromNodeToNode(xprev,yprev,r,xn,yn,r);
      glLineWidth(2);
    }

    xprev = xprev + xstep*selected_node;
    yprev = yn;
  }
  glDisable(GL_BLEND);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_LIGHTING);

}

void ViewLocalMinimaTree::DrawNode( double x, double y, double radius, int number)
{
    DrawNode( x, y, radius, std::to_string(number));
}

void ViewLocalMinimaTree::DrawNode( double x, double y, double radius, std::string nstr)
{

  int numIncrements = 16;
  nodeColor.setCurrentGL();
  float inc=Pi*Two/numIncrements;
  Complex cx(radius,0),dx;
  dx.setPolar(One,inc);

  glBegin(GL_TRIANGLE_FAN);
  glVertex2f(x,y);
  for(int i=0;i<=numIncrements;i++) {
    glVertex2f(x+cx.x,y-cx.y);
    cx = cx*dx;
  }
  glEnd();
  boundColor.setCurrentGL();
  glBegin(GL_LINE_LOOP);
  for(int i=0;i<=numIncrements;i++) {
    glVertex2f(x+cx.x,y-cx.y);
    cx = cx*dx;
  }
  glEnd();

  textColor.setCurrentGL();
  char buf[64];
  void* font=GLUT_BITMAP_HELVETICA_18;
  sprintf(buf,"%s\n", nstr.c_str());
  glRasterPos2d(x-5,y+5);
  glutBitmapString(font,buf);


}
void ViewLocalMinimaTree::DrawLineFromNodeToNode(double x1, double y1, double r1, double x2, double y2, double r2)
{
  Vector2 v(x2-x1,y2-y1);
  v = v/v.norm();
  double r3 = std::max(node_radius_selected, node_radius_unselected);
  double x3 = x1;
  double y3 = y1+r3;
  double x4 = x2;
  double y4 = y2-r2;
  double y34 = y3 + 0.5*(y4-y3);
  glBegin(GL_LINE_STRIP);
  glVertex2i(x3, y3);
  glVertex2i(x3, y34);
  glVertex2i(x4, y34);
  glVertex2i(x4, y4);
  glEnd();
}
