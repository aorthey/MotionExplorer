#pragma once

struct WorkspaceApproximationElement{
  Vector3 pos;
  Vector3 ori;
  double inner_radius;
  double outer_radius;
};
struct WorkspaceApproximation{
  std::vector<WorkspaceApproximationElement> elements;
};

class PlannerOutput{

  private:
    double time;
    uint nodes;

    //tree swath
    SerializedTree _stree;

    //path
    std::vector<Config> q;
    std::vector<Config> dq;
    std::vector<Config> ddq;
    std::vector<Vector> torques;

  public:

    uint robot_idx;

    WorkspaceApproximation workspace;
    PlannerOutput(){};

    void SetTorques(std::vector<Vector> &torques_){
      torques = torques_;
    }
    const std::vector<Vector>& GetTorques(){
      return torques;
    }

    Config GetInitConfiguration(){
      if(q.empty()){
        std::cout << "[PlannerOutput] No path in output, cannot get initial configuration" << std::endl;
        exit(0);
      }
      return q.at(0);
    }

    const SerializedTree& GetTree()
    {
      return _stree;
    }
    void SetTree(SerializedTree &stree)
    {
      _stree = stree;
    }

    const std::vector<Config> GetKeyframes(){
      return q;
    }
    void SetKeyframes(std::vector<Config> &keyframes){
      q = keyframes;
    }

};

