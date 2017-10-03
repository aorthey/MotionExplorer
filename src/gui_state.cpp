#include "gui_state.h"
#include <regex>

GUIVariable::GUIVariable(){
  name = "unknown";
}
GUIVariable::GUIVariable(std::string name_){
  name = name_;

  descr = std::string(name);
  descr = std::regex_replace(descr, std::regex("_"), " ");
}
GUIVariable::GUIVariable(std::string name_, std::string descr_){
  name = name_;
  descr = descr_;
}

void GUIVariable::toggle(){
  if(active) active=false;
  else active=true;
}

GUIVariable::operator bool() const{
  return active;
}
bool GUIVariable::operator!() const{
  return !active;
}

GUIState::GUIState(){
}

void GUIState::add(const char* name){
  GUIVariable* v = new GUIVariable(name);
  v->active = 0;
  variables[v->name] = v;
}
void GUIState::add(const char* name, char* key){
  GUIVariable*  v = new GUIVariable(name);
  v->active = 0;
  v->key = key;
  variables[v->name] = v;
}
GUIVariable& GUIState::operator()(const char* str){
  //return variables.at(std::string(str));
  return *variables[std::string(str)];
  //return variables.find(std::string(str))->second;
}
void GUIState::toggle(const char* str){
  //(*this)(str).toggle();
}
bool GUIState::load(const char* file)
{
  TiXmlDocument doc(file);
  TiXmlElement *root = GetRootNodeFromDocument(doc);
  return load(root);
}
bool GUIState::load(TiXmlElement *node)
{
  CheckNodeName(node, "gui");
  TiXmlElement* node_state = FindSubNode(node, "state");

  while(node_state!=NULL){
    GUIVariable* v = new GUIVariable();

    TiXmlElement* node_name = FindSubNode(node_state, "name");
    TiXmlElement* node_descr = FindSubNode(node_state, "descr");
    TiXmlElement* node_key = FindSubNode(node_state, "key");
    TiXmlElement* node_active = FindSubNode(node_state, "active");
    if(node_name) GetStreamText(node_name) >> v->name;
    if(node_descr) std::getline(GetStreamText(node_descr), v->descr);
    if(node_key) GetStreamText(node_key) >> v->key;
    if(node_active) GetStreamText(node_active) >> v->active;

    variables[v->name] = v;
    node_state = FindNextSiblingNode(node_state, "state");
  }
  return true;
}

std::ostream& operator<< (std::ostream& out, const GUIVariable& v)
{
  out << "  <state>" << std::endl;
  out << "    <name>" << v.name << "</name>" << std::endl;
  out << "    <descr>" << v.descr << "</descr>" << std::endl;
  out << "    <key>" << v.key << "</key>" << std::endl;
  out << "    <active>" << v.active << "</active>" << std::endl;
  out << "  </state>" << std::endl;
  return out;
}
std::ostream& operator<< (std::ostream& out, const GUIState& s)
{

  std::cout << std::string(80, '-') << std::endl;
  std::cout << "<gui>" << std::endl;
  std::cout << std::string(80, '-') << std::endl;
  for (auto it = s.variables.begin(); it != s.variables.end(); ++it) {
    std::cout << it->second;
  }
  std::cout << "</gui>" << std::endl;
  std::cout << std::string(80, '-') << std::endl;
  return out;
}
