#pragma once

class PlannerOutput{

  private:
    double time;
    uint nodes;
    std::vector<Config> keyframes;

  public:
    PlannerOutput(){};

    std::vector<Config> GetRoadmapVertices();
    std::vector<std::pair<Config,Config> > GetRoadmapEdges();

    std::vector<Config> GetPath();
    Config GetInitialConfig();
    Config GetGoalConfig();

};

