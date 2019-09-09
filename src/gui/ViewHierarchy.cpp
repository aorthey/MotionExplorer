#include "ViewHierarchy.h"

using namespace Math3D;
ViewHierarchy::ViewHierarchy():
  x(0),y(0),heightPerLevel(50),width(300),width_column1(30),width_column2(50),width_column3(width-width_column1-width_column2),drawBound(true),drawPlotArea(true)
{
  textColor.set(0.1,0.1,0.1);
  nodeColor.set(0.8,0.8,0.8);
  nodeSelectedColor.set(0.4,0.4,0.4);
  nodeUnselectedColor.set(0.8,0.8,0.8);
  boundColor.set(0.2,0.2,0.2);
  plotAreaColor.set(0.8,0.8,0.8,0.5);
}
void ViewHierarchy::PushLevel(int nodes, std::string robot_name){
  if(nodes>0){
    level_nodes.push_back(nodes);
    selected_path.push_back(0);
    level_robot_name.push_back(robot_name);
  }
}
void ViewHierarchy::UpdateSelectionPath( std::vector<int> path ){
  if(path.size() == selected_path.size()){
    selected_path = path;
  }else{
    std::cout << "[ViewHierarchy.h] Updating selection with invalid path" << std::endl;
    std::cout << "input path: " << std::endl;
    for(uint k = 0; k < path.size(); k++){
      std::cout << "  node " << path.at(k) << std::endl;
    }
    std::cout << "levels: " << std::endl;
    for(uint k = 0; k < level_nodes.size(); k++){
      std::cout << "  level " << k << " nodes " << level_nodes.at(k) << std::endl;
    }
    std::cout << "old selected path:" << std::endl;
    for(uint k = 0; k < selected_path.size(); k++){
      std::cout << "  node " << selected_path.at(k) << std::endl;
    }
    throw "Error selection path.";
  }
}
void ViewHierarchy::PopLevel()
{
  if(level_nodes.size()>0){
    level_nodes.erase( level_nodes.end() - 1);
    level_robot_name.erase( level_robot_name.end() - 1);
    selected_path.erase( selected_path.end() - 1);
  }
}

int ViewHierarchy::GetLevel()
{
  return level_nodes.size();
}
void ViewHierarchy::Clear()
{
  level_nodes.clear();
  selected_path.clear();
  level_robot_name.clear();
}

void ViewHierarchy::DrawGL(){

  double xprev = x+width_column1+width_column2+0.5*width_column2;
  double yprev = y+heightPerLevel/2;
  double node_radius = heightPerLevel/3;
  double xstep = node_radius*2.5;
  double node_radius_unselected = node_radius - node_radius/4;
  double node_radius_selected = node_radius;

  double height = heightPerLevel * (GetLevel()+1);

  int maxNodes = 1;
  int addNodes = 0;
  for(uint k = 0; k < level_nodes.size(); k++){
    std::vector<int> selected_path;
    int nk = level_nodes.at(k) + addNodes;
    if(nk > maxNodes) maxNodes = nk;
    if(k < selected_path.size()){
      addNodes += selected_path.at(k);
    }
  }
  double width = xprev + maxNodes * xstep;
  width_column3 = width-width_column1-width_column2;

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
    glBegin(GL_LINE_LOOP);
    glVertex2i(x+width_column1,y);
    glVertex2i(x+width_column1,y+height);
    glEnd();
    glBegin(GL_LINE_LOOP);
    glVertex2i(x+width_column1+width_column2,y);
    glVertex2i(x+width_column1+width_column2,y+height);
    glEnd();
    glBegin(GL_LINE_LOOP);
    glVertex2i(x+width_column1+width_column2+width_column3,y);
    glVertex2i(x+width_column1+width_column2+width_column3,y+height);
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
    double yn = y+(k+1)*heightPerLevel+heightPerLevel/2;

    uint nodes_on_level = level_nodes.at(k);
    uint selected_node = selected_path.at(k);

    for(uint i = 0; i < nodes_on_level; i++){
      double xn = xprev + xstep*i;
      double r = (i==selected_node? node_radius_selected: node_radius_unselected);
      nodeColor = (i==selected_node? nodeSelectedColor:nodeUnselectedColor);
      DrawNode(xn,yn,r,i);
      (i==selected_node? glLineWidth(10): glLineWidth(5));
      DrawLineFromNodeToNode(xprev,yprev,r,xn,yn,r);
      glLineWidth(5);
    }

    xprev = xprev + xstep*selected_node;
    yprev = yn;
  }

  textColor.setCurrentGL();
  for(uint k = 0; k < level_nodes.size()+1; k++){
    double yk = y+k*heightPerLevel+heightPerLevel/2;
    char buf[64];
    void* font=GLUT_BITMAP_HELVETICA_18;
    sprintf(buf,"%d\n",k);
    glRasterPos2d(x+width_column1/3,yk);
    glutBitmapString(font,buf);

    if(k>0){
      sprintf(buf,"%d\n",level_nodes.at(k-1));
    }else{
      sprintf(buf,"%d\n",1);
    }
    glRasterPos2d(x+width_column1+width_column2/3,yk);
    glutBitmapString(font,buf);

  }

  glDisable(GL_BLEND);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_LIGHTING);

}

void ViewHierarchy::DrawNode( double x, double y, double radius, int number)
{
    DrawNode( x, y, radius, std::to_string(number));
}
void ViewHierarchy::DrawNode( double x, double y, double radius, std::string nstr)
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
  glRasterPos2d(x-5,y+5);  // if(number<10){
  //   glRasterPos2d(x-5,y+5);
  // }else{
  //   glRasterPos2d(x-10,y+5);
  // }
  glutBitmapString(font,buf);


}
void ViewHierarchy::DrawLineFromNodeToNode(double x1, double y1, double n1, double x2, double y2, double n2){
  Vector2 v(x2-x1,y2-y1);
  v = v/v.norm();
  glBegin(GL_LINE_LOOP);
  glVertex2i(x1+n1*v[0],y1+n1*v[1]);
  glVertex2i(x2-n2*v[0],y2-n2*v[1]);
  glEnd();

}
  // //experiments on drawing 3d structures on fixed screen position
  // double w = viewport.w;
  // double h = viewport.h;

  // Vector3 campos = viewport.position();
  // Vector3 camdir;
  // viewport.getViewVector(camdir);

  // Vector3 up,down,left,right;

  // viewport.getClickVector(0,h/2,left);
  // viewport.getClickVector(w,h/2,right);
  // viewport.getClickVector(w/2,0,down);
  // viewport.getClickVector(w/2,h,up);

  // up = up+campos;
  // down = down+campos;
  // left = left+campos;
  // right = right+campos;

  // glLineWidth(10);
  // glPointSize(20);
  // GLColor black(1,0,0);
  // black.setCurrentGL();
  // GLDraw::drawPoint(campos);
  // GLDraw::drawPoint(up );
  // GLDraw::drawPoint(down);
  // GLDraw::drawPoint(right);
  // GLDraw::drawPoint(left);

  // GLDraw::drawLineSegment(up, left);
  // GLDraw::drawLineSegment(up, right);
  // GLDraw::drawLineSegment(down, right);
  // GLDraw::drawLineSegment(down, left);


  // GLColor grey(0.6,0.6,0.6);
  // grey.setCurrentGL();
  // glTranslate(left+0.5*(up-left));
  // GLDraw::drawSphere(0.05,16,16);

  // Vector3 tt;
  // viewport.getClickVector(w/8,h/2,tt);

  // glTranslate(tt + 0.1*camdir);
  // GLDraw::drawSphere(0.05,16,16);
  // glTranslate(tt + 0.3*camdir);
  // GLDraw::drawSphere(0.05,16,16);
  // glTranslate(tt + 0.4*camdir);
  // GLDraw::drawSphere(0.05,16,16);

  // glEnable(GL_LIGHTING);
