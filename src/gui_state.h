#include "loader.h"
#include <string>
#include <vector>
#include <map>
#include <iostream>

struct GUIVariable
{
  GUIVariable();
  GUIVariable(std::string);
  GUIVariable(std::string, std::string);

  int active;

  std::string key;
  std::string name;
  std::string descr;

  void toggle();
  operator bool() const;
  bool operator!() const;

  friend std::ostream& operator<< (std::ostream&, const GUIVariable&);
};

class GUIState{

  public:
    GUIState();
    void add(const char* name);
    void add(const char* name, char* key);
    GUIVariable& operator()(const char* str);
    void toggle(const char* str);

    friend std::ostream& operator<< (std::ostream&, const GUIState&);

    bool load(const char* file);
    bool load(TiXmlElement *node);

    std::map<std::string, GUIVariable*> variables;

};

