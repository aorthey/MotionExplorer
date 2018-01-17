#include "file_output.h"

TiXmlElement* CreateRootNodeInDocument(TiXmlDocument& doc, const char *name)
{
  TiXmlDeclaration * decl = new TiXmlDeclaration( "1.0", "", "" );
  doc.LinkEndChild(decl);
  TiXmlElement *root = new TiXmlElement(name);
  return root;
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
  for(uint k = 0; k < _val.size(); k++){
    ss << _val.at(k) << " ";
  }
  TiXmlText text(ss.str().c_str());
  subnode.InsertEndChild(text);
  node.InsertEndChild(subnode);
}
