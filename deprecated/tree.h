#pragma once

template <class T>
struct Tree
{
  Tree(T content_){
    content = content_;
  }
  T content;
  std::vector<Tree<T>* > children;
};
