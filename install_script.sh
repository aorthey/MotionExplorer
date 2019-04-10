#!/bin/bash

ubuntu_version=`lsb_release -rs | sed 's/\.//'`
if [ $ubuntu_version == "1604" ];then
  echo "Install script Ubuntu 16.04"
elif [ $ubuntu_version == "1804" ];then
  echo "Install script Ubuntu 18.04"
else
  echo "This install script is for Ubuntu {16,18}.04"
  echo "but version is $ubuntu_version"
  return 0
fi

echo "Installing dependencies"


if [ $ubuntu_version == "1604" ];then
  sudo apt-get install -qq libboost1.58-all-dev
elif [ $ubuntu_version == "1804" ];then
  sudo apt-get install -qq libboost1.65-all-dev
else
  return 0
fi

sudo apt-get install -qq g++-5 cmake git freeglut3 freeglut3-dev libglpk-dev 
sudo apt-get install -qq libxmu-dev libxi-dev libqt4-dev libeigen3-dev libassimp-dev libflann-dev liburdfdom-tools libccd-dev libqhull-dev 
sudo apt-get install -qq python-dev python-opengl python-setuptools pypy
pip install --upgrade pip

cd ~
mkdir -p git
cd ~/git
git clone git@github.com:aorthey/orthoklampt.git
cd orthoklampt
mkdir libs

echo "***********************************************************************"
echo "Installing OMPL (Planning Library)"
echo "***********************************************************************"
cd ~/git/orthoklampt/libs/
wget http://ompl.kavrakilab.org/install-ompl-ubuntu.sh
chmod u+x install-ompl-ubuntu.sh
./install-ompl-ubuntu.sh --app


echo "***********************************************************************"
echo "Installing LEMON (Graph Library)"
echo "***********************************************************************"
cd ~/git/orthoklampt/libs/
wget http://lemon.cs.elte.hu/pub/sources/lemon-1.3.1.tar.gz
tar xfv lemon-1.3.1.tar.gz 
cd lemon-1.3.1/
mkdir build
cd build/
cmake ..
make -j$(nproc)
sudo make install

echo "***********************************************************************"
echo "Installing KLAMPT (Dynamical Simulator)"
echo "***********************************************************************"
cd ~/git/orthoklampt/libs/
git clone git@github.com:aorthey/Klampt.git
cd Klampt/Library
make unpack-deps
rm -rf KrisLibrary
git clone git@github.com:aorthey/KrisLibrary.git
make deps
cd ..
cmake .
make -j$(nproc)
sudo make install

echo "***********************************************************************"
echo "Installing Orthoklampt"
echo "***********************************************************************"
cd ~/git/orthoklampt/libs/
mkdir -p build
cd build
cmake ..
make -j$(nproc)
./planner_gui ../data/experiments/15D_planar_manipulator.xml
