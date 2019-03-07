#include <vector>
#include <iostream>
#include <boost/type_traits/is_pointer.hpp>

//This script investigates how one can check INSIDE a templated function if the
//template object is a pointer or not. This can be done by using is_pointer<T>,
//but we cannot use it to handle a pointer differently (because the compiler is
//going to type-check everything before execution and will output an error ---
//it will complain that *obj is an invalid type argument, i.e. we are trying to
//go deeper into an object, which is already a primitive type). 
//A clever solution [1] is to write a ptr() function, which
//returns the object itself if it is already a pointer, or returns a reference
//to the object, to make it into a pointer. So we can write *ptr(obj) and can be
//sure that we always have a pointer at hand.
//
//[1] https://stackoverflow.com/questions/14466620/c-template-specialization-calling-methods-on-types-that-could-be-pointers-or?noredirect=1&lq=1

template<typename T>
T * ptr(T & obj) { return &obj; } //turn reference into pointer!

template<typename T>
T * ptr(T * obj) { return obj; } //obj is already pointer, return it!

template<typename T> 
std::ostream& operator<< (std::ostream& out, const std::vector<T> &v){
  const bool T_is_ptr = boost::is_pointer<T>::value;
  std::cout << (T_is_ptr?"IsPointer":"NotAPointer") << std::endl;
  for(uint k = 0; k < v.size(); k++){
    out << *ptr(v.at(k));
    out << (k<v.size()-1?" ":"");
  }
  return out;
}

struct MyDouble{
  double d;
  friend std::ostream& operator<< (std::ostream& out, const MyDouble &md)
  {
    out << "[" << md.d << "]";
    return out;
  }
};

int main(int argc,const char** argv)
{
  std::vector<double> A;
  double dstep = 0.01;
  for(double d = 0; d < 0.1; d+=dstep){
    A.push_back(d);
  }

  std::vector<MyDouble*> B;
  for(double d = 0; d < 0.1; d+=dstep){
    MyDouble *md = new MyDouble();
    md->d = d;
    B.push_back(md);
  }
  std::cout << A << std::endl;
  std::cout << B << std::endl;


  return 0;
}
