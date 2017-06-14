#pragma once

class PlannerOutput{

  private:
    double time;
    uint nodes;
    std::vector<Config> p;
    std::vector<Config> dp;
    std::vector<Vector> torques;

  public:
    PlannerOutput(){};

    //std::vector<Config> GetRoadmapVertices();
    //std::vector<std::pair<Config,Config> > GetRoadmapEdges();

    void SetTorques(std::vector<Vector> &torques_){
      torques = torques_;
    }
    const std::vector<Vector>& GetTorques(){
      return torques;
    }
    //std::vector<Config> GetPath();
    //Config GetInitialConfig();
    //Config GetGoalConfig();

};

