#pragma once

#include "util.h"
#include "gui_planner.h"
#include "info.h"
#include "planner/planner_input.h"

#include <tinyxml.h>

class EnvironmentLoader{
  private:
    std::string file_name;

    std::string name_environment;
    std::string name_robot;
    std::string name;

    RobotWorld world;
    PlannerBackendPtr _backend;
    Info info;
    PlannerMultiInput pin;

  public:
    RobotWorld& GetWorld();
    RobotWorld* GetWorldPtr();
    Robot* GetRobotPtr();
    PlannerBackendPtr GetBackendPtr();
    PlannerMultiInput GetPlannerInput();

    static EnvironmentLoader from_args(int argc,const char** argv);
    EnvironmentLoader(const char *xml_file);

    bool LoadPath(const char* file);
    bool LoadPath(TiXmlElement *node);
    std::vector<Config> GetKeyframesFromFile(const char* file);

};
