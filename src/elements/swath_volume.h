#pragma once
#include <KrisLibrary/GLdraw/drawMesh.h>
#include <KrisLibrary/GLdraw/GLError.h>
#include <KrisLibrary/GLdraw/GLColor.h>
#include <Modeling/Robot.h>

using namespace GLDraw;


// 
//Swept Volume: Volume of Robot along a path in CSpace
//Swath Volume: Volume of Robot along a graph in CSpace
//

class SwathVolume
{
  public:
    SwathVolume(Robot *robot);
    SwathVolume(Robot *robot, const std::vector<Config> &vertices);

    const std::vector<std::vector<Matrix4> >& GetMatrices();

    void SetColor(const GLColor c);
    GLColor GetColor();

    // bool Save(const char* file=NULL);
    // bool Save(TiXmlElement *node);
    // bool Load(const char* file);
    // bool Load(TiXmlElement *node);

  private:
    void AddVertex(const Config &q );

    GLColor color;
    GLColor color_milestones;

    Robot *_robot;
    std::vector<std::vector<Matrix4> > _mats;
    vector<Config> _vertices;

};


