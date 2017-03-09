#pragma once

#include <KrisLibrary/GLdraw/drawMesh.h>
#include <KrisLibrary/robotics/IK.h>
#include <KrisLibrary/robotics/IKFunctions.h>
#include <View/ViewRobot.h>
#include <View/ViewIK.h>

#include "planner.h"

namespace GLDraw {
  void drawRobotExtras(ViewRobot *robot, GLColor bodyColor=GLColor(0.5,0.5,0.5), double COMradius=0.05);
  void drawIKextras(ViewRobot *viewrobot, Robot *robot, std::vector<IKGoal> &constraints, std::vector<int> linksInCollision, GLColor selectedLinkColor);
  void drawUniformForceField();
  void drawPathSweptVolume(Robot *robot, std::vector<std::vector<Matrix4> > mats, vector<GLDraw::GeometryAppearance> appearanceStack, double sweptvolumeScale = 0.98, GLColor sweptvolumeColor = GLColor(0.7,0.0,0.9,0.5));
  void drawPlannerStartGoal(Robot *robot, const Config &p_init, const Config &p_goal);
  void drawPlannerTree(const SerializedTree &_stree);
};
