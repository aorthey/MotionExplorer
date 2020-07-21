#pragma once

#include "util.h"
#include "info.h"
#include "gui/gui_planner.h"
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
    PlannerBackendPtr GetBackendPtr();
    PlannerMultiInput GetPlannerInput();
    void RenameExec(int argc, char** argv, const std::string &s);

    void LoadController(Robot *robot, const PlannerInput &pin);

    static EnvironmentLoader from_args(int argc,char** argv);
    EnvironmentLoader(const char *xml_file);

};
