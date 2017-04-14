#pragma once
#include <KrisLibrary/GLdraw/drawMesh.h>
#include <KrisLibrary/GLdraw/GLError.h>
#include <KrisLibrary/GLdraw/GLColor.h>
#include <Modeling/Robot.h>

using namespace GLDraw;

class SweptVolume
{
  public:
    SweptVolume(Robot *robot);
    SweptVolume(Robot *robot, const std::vector<Config> &keyframes, uint Nkeyframes);
    const std::vector<std::vector<Matrix4> >& GetMatrices();
    const std::vector<Config >& GetKeyframes();
    void SetColor(const GLColor c);
    GLColor GetColor();
    void SetColorMilestones(const GLColor c);
    GLColor GetColorMilestones();
    const Config& GetStart();
    const Config& GetGoal();
    const vector<uint>& GetKeyframeIndices();
  private:
    void AddKeyframe(const Config &q );
    GLColor color;
    GLColor color_milestones;
    Robot *_robot;
    std::vector<std::vector<Matrix4> > _mats;
    vector<Config> _keyframes;
    Config init, goal;
    vector<uint> _keyframe_indices;

};

