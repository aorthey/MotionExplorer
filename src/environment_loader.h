#pragma once

#include <tinyxml.h>
#include "util.h"
#include "gui.h"
#include "info.h"
#include "planner/planner_input.h"

class EnvironmentLoader{
  private:
    std::string file_name;

    std::string name_environment;
    std::string name_robot;
    std::string name;

    RobotWorld world;
    ForceFieldBackendPtr _backend;
    Info info;
    PlannerInput pin;

  public:
    RobotWorld& GetWorld();
    RobotWorld* GetWorldPtr();
    Robot* GetRobotPtr();

    PlannerInput GetPlannerInput();
    ForceFieldBackendPtr GetBackendPtr();
    EnvironmentLoader(const char *xml_file);

    bool LoadPlannerSettings(TiXmlElement *node);
    bool LoadPlannerSettings(const char* file);

    bool LoadPath(const char* file);
    bool LoadPath(TiXmlElement *node);

};
