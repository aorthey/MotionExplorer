#pragma once

bool CheckNodeName(TiXmlElement *node, const char* name)
{
  if(0!=strcmp(node->Value(),name)) {
    std::cout << "Not a " << name <<  " file" << std::endl;
    return false;
  }
  return true;
}
TiXmlElement* GetRootNodeFromDocument(TiXmlDocument& doc)
{
  if(doc.LoadFile()){
    TiXmlElement *root = doc.RootElement();
    if(root){
      return root;
    }
  }else{
    std::cout << doc.ErrorDesc() << std::endl;
    return NULL;
  }
}

TiXmlElement* FindSubNode(TiXmlElement* node, const char *name){
  if(!node) return NULL;
  TiXmlElement* e=node->FirstChildElement();
  while(e != NULL) 
  {
    if(0==strcmp(e->Value(),name)) return e; 
    e = e->NextSiblingElement();
  }
  return NULL;
}

TiXmlElement* FindFirstSubNode(TiXmlElement* node, const char *name){
  return FindSubNode(node, name);
}

TiXmlElement* FindNextSiblingNode(TiXmlElement* node, const char *name){
  while(node != NULL) 
  {
    node = node->NextSiblingElement();
    if(node!=NULL){
      if(0==strcmp(node->Value(),name)) return node; 
    }
  }
  return NULL;
}

stringstream GetStreamAttribute(TiXmlElement* node, const char *name){

  if(!node) return stringstream ("NONE");
  const char *na = node->Attribute(name);
  if(na){
    return stringstream (na);
  }else{
    return stringstream ("NONE");
  }
}
stringstream GetStreamText(TiXmlElement* node){

  if(!node) return stringstream ("NONE");
  const char *na = node->GetText();
  if(na){
    return stringstream (na);
  }else{
    return stringstream ("NONE");
  }
}
