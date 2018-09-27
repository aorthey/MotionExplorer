#include "file_output.h"

TiXmlElement* CreateRootNodeInDocument(TiXmlDocument& doc, const char *name)
{
  TiXmlDeclaration * decl = new TiXmlDeclaration( "1.0", "", "" );
  doc.LinkEndChild(decl);
  TiXmlElement *root = new TiXmlElement(name);
  return root;
}
TiXmlElement* CreateSubNode(TiXmlElement& parent_node, const char *name)
{
  TiXmlElement *subnode = new TiXmlElement(name);
  parent_node.InsertEndChild(*subnode);
  return subnode;
}

template <typename T>
void AddSubNode(TiXmlElement& node, const char *name, T _val){

  TiXmlElement subnode(name);
  stringstream ss;
  ss<< _val;
  TiXmlText text(ss.str().c_str());
  subnode.InsertEndChild(text);
  node.InsertEndChild(subnode);
}
template <typename T>
void AddSubNodeVector(TiXmlElement& node, const char *name, std::vector<T> _val){

  TiXmlElement subnode(name);
  stringstream ss;
  ss << _val.size() << "  ";
  for(uint k = 0; k < _val.size(); k++){
    ss << _val.at(k) << " ";
  }
  TiXmlText text(ss.str().c_str());
  subnode.InsertEndChild(text);
  node.InsertEndChild(subnode);
}

template <typename T>
TiXmlElement* ReturnSubNodeVector(TiXmlElement& node, const char *name, std::vector<T> _val){

  TiXmlElement* subnode = new TiXmlElement(name);
  stringstream ss;
  ss << _val.size() << "  ";
  for(uint k = 0; k < _val.size(); k++){
    ss << _val.at(k) << " ";
  }
  TiXmlText text(ss.str().c_str());
  subnode->InsertEndChild(text);
  return subnode;
}

void AddComment(TiXmlElement& node, const char *str)
{
  TiXmlComment * comment = new TiXmlComment();
  comment->SetValue(str);
  node.LinkEndChild(comment);  
}
