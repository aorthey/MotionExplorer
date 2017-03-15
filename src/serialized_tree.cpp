#include <sstream>
#include "serialized_tree.h"

using namespace std;
Vector3 SerializedTreeNode::GetXYZ(){
  Vector3 pos;
  pos[0] = position[0];
  pos[1] = position[1];
  pos[2] = position[2];
  return pos;
}
bool SerializedTreeNode::Save(TiXmlElement *node)
{
  node->SetValue("node");

  {
    TiXmlElement cc("position");
    stringstream ss;
    ss<<position;
    TiXmlText text(ss.str().c_str());
    cc.InsertEndChild(text);
    node->InsertEndChild(cc);
  }
  {
    TiXmlElement cc("config");
    stringstream ss;
    ss<<config;
    TiXmlText text(ss.str().c_str());
    cc.InsertEndChild(text);
    node->InsertEndChild(cc);
  }
  {
    TiXmlElement cc("directions");
    for(int i = 0; i < directions.size(); i++){
      TiXmlElement ccc("d");
      stringstream ss;
      ss<<directions.at(i);
      TiXmlText text(ss.str().c_str());
      ccc.InsertEndChild(text);
      cc.InsertEndChild(ccc);
    }
    node->InsertEndChild(cc);
  }
  {
    TiXmlElement cc("cost_to_goal");
    stringstream ss;
    ss<<cost_to_goal;
    TiXmlText text(ss.str().c_str());
    cc.InsertEndChild(text);
    node->InsertEndChild(cc);
  }

}
bool SerializedTreeNode::Load(TiXmlElement *node)
{
  if(0!=strcmp(node->Value(),"node")) {
    fprintf(stderr,"SerializedTreeNode Load XML: node \"%s\" not of node type\n",node->Value());
    return false;
  }

  TiXmlElement* e=node->FirstChildElement();
  while(e != NULL) 
  {
    if(0==strcmp(e->Value(),"position")) {
      stringstream ss(e->GetText());
      ss >> position;
    }
    if(0==strcmp(e->Value(),"config")) {
      stringstream ss(e->GetText());
      ss >> config;
    }
    if(0==strcmp(e->Value(),"cost_to_goal")) {
      stringstream ss(e->GetText());
      ss >> cost_to_goal;
    }
    if(0==strcmp(e->Value(),"directions")) {
      TiXmlElement* d=e->FirstChildElement();
      while(d != NULL)
      {
        if(d->GetText()!=NULL) {
          stringstream ss(d->GetText());
          Vector3 dir;
          ss >> dir;
          directions.push_back(dir);
          d = d->NextSiblingElement();
        }
      }
    }
    e = e->NextSiblingElement();
  }
  return true;

}
