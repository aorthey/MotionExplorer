#include <iostream>
#include <vector>

struct Train
{
  std::string name;
  int dim{0};

  Train(std::string name): name(name){}
  Train(std::string name, int dim): name(name), dim(dim){}
  virtual void print(){ std::cout << name << std::endl; }
  virtual bool isContainerTrain(){ return false; }
  int getDimension(){
    return dim;
  }
};

struct ContainerTrain: public Train
{
  ContainerTrain(): Train("containertrain", 18) {}
  virtual void unload(){ std::cout << "Unloaded Train. Dim=" << dim << std::endl; }
  bool isContainerTrain() override{ return true; }
};

struct MultiTrain: public Train
{
  MultiTrain(std::vector<Train*> trains): Train("multitrain", 20),
    trains_(trains)
  {
  }
  void print() override
  { 
    std::cout << "MultiTrain:" << std::endl;
    for(uint k = 0; k < trains_.size(); k++)
    {
      std::cout << " --" << trains_.at(k)->name <<
        (trains_.at(k)->isContainerTrain()?" (container) ":"") << std::endl;
    }
  }
  std::vector<Train*> trains_;
};

struct MultiContainerTrain: public ContainerTrain, public MultiTrain
{
  MultiContainerTrain(std::vector<Train*> trains): 
    ContainerTrain(), MultiTrain(trains)
  {
    ContainerTrain::name = "multicontainertrain";
    MultiTrain::name = "multicontainertrain";
  }

  void print() override{ MultiTrain::print(); }

  bool isContainerTrain() override{ 
    for(uint k = 0; k < trains_.size(); k++)
    {
      if(!trains_.at(k)->isContainerTrain()) return false;
    }
    return true; 
  }
};

int main()
{
    ContainerTrain *ct1 = new ContainerTrain();
    ContainerTrain *ct2 = new ContainerTrain();
    std::vector<Train*> trains;
    trains.push_back(ct1);
    trains.push_back(ct2);
    ContainerTrain *tm = new MultiContainerTrain(trains);
    std::cout << (tm->isContainerTrain()?"CONTAINER":"") << std::endl;
    tm->print();
    tm->unload();
    return 0;
}

