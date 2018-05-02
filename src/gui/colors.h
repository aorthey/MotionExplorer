#pragma once
#include <KrisLibrary/GLdraw/GLColor.h>
typedef GLDraw::GLColor GLColor;

const GLColor red(1,0,0,1);
const GLColor green(0,1,0,1);
const GLColor blue(0,0,1,1);
const GLColor lightred(0.8,0,0,0.5);
const GLColor lightgreen(0,0.8,0,0.5);
const GLColor lightblue(0,0,0.8,0.5);

const GLColor yellow(1,1,0,0.5);
const GLColor magenta(1,0,1,0.5);
const GLColor cyan(0,1,1,0.5);

const GLColor orange(1,0.5,0,0.5);
const GLColor purple(0.5,0,0.5,0.5);

const GLColor black(0,0,0,0.5);
const GLColor grey(0.7,0.7,0.7,0.5);
const GLColor gray(0.7,0.7,0.7,0.5);
const GLColor lightGrey(0.4,0.4,0.4,0.2);
const GLColor cRobot(0.4,0.4,0.4,0.5);
const GLColor lightGreen(0.2,0.9,0.2,0.8);
const GLColor lightRed(0.9,0.2,0.2,0.8);

namespace GLDraw{
  void setColor(const GLColor &c);
};
