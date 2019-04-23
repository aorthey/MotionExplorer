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
  std::cout << std::string(80, '-') << std::endl;
  std::cout << (T_is_ptr?"IsPointer":"NotAPointer") << std::endl;
  std::cout << std::string(80, '-') << std::endl;
  for(uint k = 0; k < v.size(); k++){
    out << *ptr(v.at(k));
    out << (k<v.size()-1?" ":"");
  }
  return out;
}

struct MyDouble{
  double d;
  MyDouble(double d_): d(d_){};
  friend std::ostream& operator<< (std::ostream& out, const MyDouble &md)
  {
    out << "[" << md.d << "]";
    return out;
  }
};

int main(int argc,const char** argv)
{
  //A is double vector, B is MyDouble* vector, C is int vector

  std::vector<double> A{0.01,0.8,0.1,0.13,13.5};
  std::vector<MyDouble*> B;
  for(double d = 0; d < 0.1; d+=0.01){
    B.push_back(new MyDouble(d));
  }
  std::vector<int> C{0,1,3,2};

  std::cout << A << std::endl;
  std::cout << B << std::endl;
  std::cout << C << std::endl;


  return 0;
}
