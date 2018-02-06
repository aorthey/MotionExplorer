#pragma once
#include "gui/gui_state.h"
#include "gui/colors.h"
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

    const std::vector<std::vector<Matrix4> >& GetMatrices() const;
    const std::vector<Config >& GetKeyframes() const;
    const vector<GLDraw::GeometryAppearance>& GetAppearanceStack() const;

    void SetColor(const GLColor c);
    GLColor GetColor() const;
    void SetColorMilestones(const GLColor c);
    GLColor GetColorMilestones() const;
    const Config& GetStart() const;
    const Config& GetGoal() const;
    const vector<uint>& GetKeyframeIndices() const;

    Robot* GetRobot() const;

    bool Save(const char* file=NULL);
    bool Save(TiXmlElement *node);
    bool Load(const char* file);
    bool Load(TiXmlElement *node);

    void DrawGL(GUIState& state);

  protected:
    void AddKeyframe(Config &q );

    double sweptvolumeScale{1.0};
    GLColor color{grey};

    Robot *_robot;
    std::vector<std::vector<Matrix4> > _mats;
    vector<Config> _keyframes;
    vector<GLDraw::GeometryAppearance> _appearanceStack;

    Config init, goal;
    vector<uint> _keyframe_indices;

};

