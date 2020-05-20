#include "colors.h"
using namespace GLDraw;
void GLDraw::setColor(const GLColor &c){
  c.setCurrentGL();
}
int colorType = 1;
const GLColor GLDraw::getColorRobotStartConfiguration()
{
  // const GLColor colorStartConfiguration(0.7, 0.7, 0.7, 1);
  // return colorStartConfiguration;
  return GLColor(0.3, 1.0, 0.3, 1.0);
}
const GLColor GLDraw::getColorRobotStartConfigurationTransparent()
{
  // const GLColor colorStartConfiguration(0.7, 0.7, 0.7, 0.1);
  // return colorStartConfiguration;
  return GLColor(0.3, 1.0, 0.3, 0.3);
}
const GLColor GLDraw::getColorRobotGoalConfiguration()
{
  // const GLColor colorGoalConfiguration(1.0, 1.0, 0.0, 0.5);
  // return colorGoalConfiguration;
  return GLColor(0.7, 0.3, 0.3, 1.0);
}
const GLColor GLDraw::getColorRobotGoalConfigurationTransparent()
{
  // const GLColor colorGoalConfiguration(1.0, 1.0, 0.0, 0.1);
  // return colorGoalConfiguration;
  return GLColor(0.7, 0.3, 0.3, 0.3);
}
