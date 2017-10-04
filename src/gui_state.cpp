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
void GUIVariable::deactivate(){
  active = false;
}
void GUIVariable::activate(){
  active = true;
}

void GUIVariable::toggle(){
  if(active) active=false;
  else active=true;
}

GUIVariable::operator bool() const{
  return active;
}
bool GUIVariable::hasKey() const{
  if(key == "NONE") return false;
  else return true;
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
  if ( variables.find(str) == variables.end() ) {
    if(!EMPTY_VARIABLE){
      EMPTY_VARIABLE = new GUIVariable("empty");
      EMPTY_VARIABLE->active=0; 
      EMPTY_VARIABLE->descr="empty variable"; 
    }
    return *EMPTY_VARIABLE;
  }
  return *variables[std::string(str)];
}
void GUIState::toggle(const char* str){
  (*this)(str).toggle();
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


  //################################################################################
  //checkbox states
  //################################################################################
  TiXmlElement* node_state = FindSubNode(node, "checkbox");
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

    v->type = GUIVariable::Type::CHECKBOX;
    variables[v->name] = v;
    node_state = FindNextSiblingNode(node_state, "checkbox");
  }

  //################################################################################
  //hotkey variables
  //################################################################################
  node_state = FindSubNode(node, "hotkey");

  while(node_state!=NULL){
    GUIVariable* v = new GUIVariable();

    TiXmlElement* node_name = FindSubNode(node_state, "name");
    TiXmlElement* node_descr = FindSubNode(node_state, "descr");
    TiXmlElement* node_key = FindSubNode(node_state, "key");
    if(node_name) GetStreamText(node_name) >> v->name;
    if(node_descr) std::getline(GetStreamText(node_descr), v->descr);
    if(node_key) GetStreamText(node_key) >> v->key;

    v->type = GUIVariable::Type::HOTKEY;
    variables[v->name] = v;
    node_state = FindNextSiblingNode(node_state, "hotkey");
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
  //get all keys and sort them lexicographically
  std::map< std::string, std::string > keymap;

  for (auto it = s.variables.begin(); it != s.variables.end(); ++it) {
    const GUIVariable* v = it->second;
    if(v->hasKey()){
      std::string s = v->key + ": " + v->descr;
      keymap[v->key] = s;
    }
  }
  for (auto it = keymap.begin(); it != keymap.end(); ++it) {
    std::cout << it->second << std::endl;
  }
  return out;
}
