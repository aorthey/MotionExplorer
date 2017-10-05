#pragma once

class PathSpaceDecoratorSweptVolumePath: public PathSpaceDecorator{
  public:
    PathSpaceDecoratorSweptVolumePath(PathSpace* space_):
      PathSpaceDecorator(space_)
    {
    }
    void DrawGL(const GUIState&){
      uint ridx = input.robot_idx;
      Robot* robot = world->robots[ridx];
      const Config qi_in = input.q_init;
      const Config qg_in = input.q_goal;

      GLColor lightGrey(0.4,0.4,0.4,0.2);
      GLColor lightGreen(0.2,0.9,0.2,0.2);
      GLColor lightRed(0.9,0.2,0.2,0.2);
      GLColor magenta(0.9,0.1,0.9,0.5);

      GLDraw::drawRobotAtConfig(robot, qi_in, lightGreen);
      GLDraw::drawRobotAtConfig(robot, qg_in, lightRed);

      const std::vector<Config> path = component->GetShortestPath();
      if(path.size()>0){
        GLDraw::drawPath(path, magenta, 10);
        const SweptVolume& sv = component->GetSweptVolume(robot);
        GLDraw::drawGLPathSweptVolume(sv.GetRobot(), sv.GetMatrices(), sv.GetAppearanceStack(), sv.GetColor());
      }
    }
};

