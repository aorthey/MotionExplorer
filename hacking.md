# Conventions 

<!--- ------------------------------------------------------------------------->

## Naming Conventions

[CamelCase]  (https://en.wikipedia.org/wiki/Camel_case)

[snake_case] (https://en.wikipedia.org/wiki/Snake_case)

[kebab-case] (https://en.wikipedia.org/wiki/Kebab_case)

* Class        : start uppercase, rest camelcase (MyClass)

* Functions    : start uppercase, rest camelcase (MyNewFunction)

* variables    : start lowercase, rest camelcase (myVariableName)

* filenames    : all lowercase  , rest snakecase (my_file_name_convention.h)

* XML naming   : all lowercase  , rest snakecase (<robot_names/>)

<!--- ------------------------------------------------------------------------->
## Formatting Conventions

* Identation   : 2 whitespaces, no tabs

* Curly Braces : on new line (no egyptian, see below)

<!--- ------------------------------------------------------------------------->
## Git Commit Message Convention

* Written in imperative. Right: "Fix bug #3", Wrong: "Fixed bug #3". 

* Rule of thumb: Message should complete sentence "This commit will ..."

* Capitalize first letter.

<!--- ------------------------------------------------------------------------->
## Class Conventions

Every class should be writeable to the terminal, to the screen and to a file.
Every class should be readable from a file. Writing to the terminal allows one
to quickly check the content of the class, writing to the screen provides a
visual aid, and writing to a file allows one to save the whole program in a
single XML file. Reading from a file allows one to restore the whole program at
any instance in time.

In short, every Object should be writeable 
  1. to the terminal : std::cout << obj << std::endl;
  2. to the screen   : obj.DrawGL(state)
  3. to a file (xml) : obj.Save(filename) 

Every object should be readable 
  1. from a file     : obj.Load(filename)

Whenever a write does not make sense, implement empty function body

### Example of minimal Object

``` c++
//object.h
#pragma once
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
virtual friend, which is not allowed in cpp. A good solution is proposed by  
https://en.wikibooks.org/wiki/More_C++_Idioms/Virtual_Friend_Function :
Implement the operator<< only on the parent class, and let it call a virtual function
Print, which then can be overriden as usual.

### Example of operator<< inheritance pattern

``` c++
//object.h
#pragma once
class Object
{
  friend std::ostream& operator<< (std::ostream& out, const Object& obj);
  virtual void Print(std::ostream &out) const;
}
class SubObject: Object
{
  virtual void Print(std::ostream &out) const override;
}
``` 
``` c++
//object.cpp
std::ostream& operator<< (std::ostream& out, const Object& obj) 
{
  obj.Print(out);
  return out;
}
void Object::Print(std::ostream &out) const
{
  out << std::string(80, '-') << std::endl;
  out << "[Object] " << std::endl;
  out << std::string(80, '-') << std::endl;
}
void SubObject::Print(std::ostream &out) const
{
  out << std::string(80, '-') << std::endl;
  out << "[SubObject] " << std::endl;
  out << std::string(80, '-') << std::endl;
}
``` 

<!--- ------------------------------------------------------------------------->
## Inherited Classes Conventions

When inheriting, create a base type BaseT to denote the super class

``` c++
//subobject.h
#pragma once
class SubObject: Object
{
  typedef BaseT Object;
}
```


<!--- ------------------------------------------------------------------------->
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

<!--- ------------------------------------------------------------------------->
## data/ Folder Convention (Naming Conventions apply)

Robots        as URDF   in data/robots/
Environments  as .tri   in data/environments/
Objects       as .tri   in data/environments/

World Files   as .xml   in data/

Format World Files: robot_environment.xml or robot_environment_comment.xml

