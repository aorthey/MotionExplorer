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
    const vector<GLDraw::GeometryAppearance>& GetAppearanceStack();

    void SetColor(const GLColor c);
    GLColor GetColor();
    void SetColorMilestones(const GLColor c);
    GLColor GetColorMilestones();
    const Config& GetStart();
    const Config& GetGoal();
    const vector<uint>& GetKeyframeIndices();

    Robot* GetRobot();

    bool Save(const char* file=NULL);
    bool Save(TiXmlElement *node);
    bool Load(const char* file);
    bool Load(TiXmlElement *node);

  protected:
    void AddKeyframe(const Config &q );

    GLColor color;
    GLColor color_milestones;

    Robot *_robot;
    std::vector<std::vector<Matrix4> > _mats;
    vector<Config> _keyframes;
    vector<GLDraw::GeometryAppearance> _appearanceStack;

    Config init, goal;
    vector<uint> _keyframe_indices;

};

