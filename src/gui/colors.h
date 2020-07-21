#pragma once
#include <KrisLibrary/GLdraw/GLColor.h>
typedef GLDraw::GLColor GLColor;

const GLColor red(1,0.3,0.3,1);
const GLColor green(0.3,1,0.3,1);
const GLColor blue(0,0,1,1);

const GLColor yellow(1,1,0,1);
const GLColor magenta(1,0,1,1);
const GLColor cyan(0,1,1,0.5);

const GLColor orange(1,0.5,0,1);
const GLColor purple(0.5,0,0.5,1);

const GLColor black(0,0,0,0.5);
const GLColor white(1,1,1,1);
const GLColor grey(0.7,0.7,0.7,0.5);
const GLColor gray(0.7,0.7,0.7,0.5);
const GLColor cRobot(0.4,0.4,0.4,0.5);

const GLColor lightGrey(0.4,0.4,0.4,0.2);
const GLColor lightGreen(0.5,0.9,0.5,0.5);
const GLColor lightRed(0.9,0.2,0.2,0.3);
const GLColor lightBlue(0,0,0.8,0.5);
const GLColor lightMagenta(0.9,0.3,0.9,0.3);
const GLColor lightOrange(0.7,0.3,0,0.3);

const GLColor darkMagenta(0.7,0,0.7,1);
const GLColor darkRed(0.5,0,0,1);
const GLColor darkGreen(0,0.5,0,1);
const GLColor darkBlue(0,0,0.5,1);

namespace GLDraw{
  const GLColor getColorRobotStartConfiguration();
  const GLColor getColorRobotGoalConfiguration();
  const GLColor getColorRobotStartConfigurationTransparent();
  const GLColor getColorRobotGoalConfigurationTransparent();
  void setColor(const GLColor &c);
};
