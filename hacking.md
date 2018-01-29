# Conventions 

<--! ------------------------------------------------------------------------->
## Naming Conventions

*CamelCase  (https://en.wikipedia.org/wiki/Camel_case)
*snake_case (https://en.wikipedia.org/wiki/Snake_case)
*kebab-case (https://en.wikipedia.org/wiki/Kebab_case)

Class        : start uppercase, rest camelcase (MyClass)
Functions    : start uppercase, rest camelcase (MyNewFunction)
variables    : start lowercase, rest camelcase (myVariableName)
filenames    : all lowercase  , rest snakecase (my_file_name_convention.h)
XML naming   : all lowercase  , rest snakecase (<robot_names/>)

<--! ------------------------------------------------------------------------->
## Formatting Conventions

Identation   : 2 whitespaces, no tabs
Curly Braces : on new line (no egyptian, see below)

<--! ------------------------------------------------------------------------->
## Class Conventions

Objective: It should be possible to save the whole program in a single XML
file, and be able to load it again. If Save() of an Object is called, then the
XML should be created recursively from all its member objects.

class Object;
Object obj;

Every Object should be writeable 
  (1) to the terminal : std::cout << obj << std::endl;
  (2) to the screen   : obj.DrawGL(state)
  (3) to a file (xml) : obj.Save(filename) 

Every object should be readable 
  (1) from a file     : obj.Load(filename)

Whenever the write does not make sense, implement empty function body

### Example

``` c++
//object.h
class Object
{
    bool Load(const char *fn);
    bool Load(TiXmlElement* node);
    bool Save(const char *fn);
    bool Save(TiXmlElement* node);
    void DrawGL(GUIState& state);
    friend std::ostream& operator<< (std::ostream& out, const Object& obj);
}
```

``` c++
//object.cpp
bool Object::Load(const char* fn)
{
  TiXmlDocument doc(file);
  return load(GetRootNodeFromDocument(doc));
}
bool Object::Load(TiXmlElement *node)
{
  CheckNodeName(node, "object");
}
bool Object::Save(const char* file)
{
  TiXmlDocument doc;
  TiXmlElement *node = CreateRootNodeInDocument(doc);
  Save(node);
  doc.LinkEndChild(node);
  doc.SaveFile(out.c_str());
  return true;
}

bool Object::Save(TiXmlElement *node)
{
  node->SetValue("object");
}
std::ostream& operator<< (std::ostream& out, const Object& obj) 
{
  out << std::string(80, '-') << std::endl;
  out << "[Object] " << std::endl;
  out << std::string(80, '-') << std::endl;
  return out;
}

``` 

Sometimes, the operator<< must be overloaded by an inherting class (for
example to implement the Decorator pattern). However, this would require a
virtual friend, which is not allowed in cpp. See
https://en.wikibooks.org/wiki/More_C++_Idioms/Virtual_Friend_Function for a
workaround.

<--! ------------------------------------------------------------------------->
## Templated Classes Conventions

Declaration file    : templated.h
Implementation file : templated.ipp

templated.h
``` c++
#pragma once

template <typename T>
void Function(T value);

#include "templated.ipp"
```

templated.ipp
``` c++
#include "templated.h"

template <typename T>
void Function(T value){
  std::cout << value << std::endl;
}
```

#include "templated.h"

<--! ------------------------------------------------------------------------->
## data/ Folder Convention (Naming Conventions apply)

Robots        as URDF   in data/robots/
Environments  as .tri   in data/environments/
Objects       as .tri   in data/environments/

World Files   as .xml   in data/

Format World Files: robot_environment.xml or robot_environment_comment.xml

