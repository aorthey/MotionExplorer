#pragma once

#include <KrisLibrary/GLdraw/drawMesh.h>
#include <KrisLibrary/robotics/IK.h>
#include <KrisLibrary/robotics/IKFunctions.h>
#include <KrisLibrary/camera/viewport.h>
#include <View/ViewRobot.h>
#include <View/ViewIK.h>

#include "planner/planner.h"

namespace GLDraw {
  void drawRobotExtras(ViewRobot *robot, GLColor bodyColor=GLColor(0.5,0.5,0.5), double COMradius=0.05);
  void drawIKextras(ViewRobot *viewrobot, Robot *robot, std::vector<IKGoal> &constraints, std::vector<int> linksInCollision, GLColor selectedLinkColor);
  void drawUniformForceField();
  void drawPathSweptVolume(Robot *robot, std::vector<std::vector<Matrix4> > mats, vector<GLDraw::GeometryAppearance> appearanceStack, double sweptvolumeScale = 0.98, GLColor sweptvolumeColor = GLColor(0.7,0.0,0.9,0.5));

  void drawPathKeyframes(Robot *robot, std::vector<uint> keyframe_indices, std::vector<std::vector<Matrix4> > mats, vector<GLDraw::GeometryAppearance> appearanceStack, double scale = 1.0, GLColor color = GLColor(0.8,0.8,0.8,1.0));

  //void drawPathMilestones(Robot *robot, std::vector<Config> &keyframes, uint Nmilestones);
  void drawPlannerStartGoal(Robot *robot, const Config &p_init, const Config &p_goal);
  void drawRobotAtConfig(Robot *robot, const Config &q, GLColor color=GLColor(1,0,0), double scale = 1.0);

  void drawPlannerTree(const SerializedTree &_stree);
  void drawAxesLabels(Camera::Viewport& viewport);
  void drawFrames(std::vector< std::vector<Vector3> > &frames, double frameLength);
};
