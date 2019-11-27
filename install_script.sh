#!/bin/bash

INSTALL_DIR=`pwd`
echo "***********************************************************************"
echo "Install Directory: ${INSTALL_DIR}"
echo "***********************************************************************"

#REQUIREMENT: 
# -- github.com account
# -- SSH access at github.com

ssh -q -T git@github.com
if [ $? -eq 255 ];then
  echo "########################################################################"
  echo "ERROR: SSH not added to GITHUB."
  echo "ERROR: https://help.github.com/en/enterprise/2.15/user/articles/adding-a-new-ssh-key-to-your-github-account"
  echo "########################################################################"
  exit 0
fi

ubuntu_version=`lsb_release -rs | sed 's/\.//'`
if [ $ubuntu_version == "1604" ];then
  echo "Install script Ubuntu 16.04"
elif [ $ubuntu_version == "1804" ];then
  echo "Install script Ubuntu 18.04"
elif [ $ubuntu_version == "1910" ];then
  echo "Install script Ubuntu 19.10"
else
  echo "This install script is for Ubuntu {16,18}.04"
  echo "but version is $ubuntu_version"
  exit 0
fi

echo "If install directory is not correct, please change it in the script before
continuing."
read -p "Press [enter] to continue"

mkdir -p ${INSTALL_DIR}
cd ${INSTALL_DIR}
echo "Installing dependencies"

if [ $ubuntu_version == "1604" ];then
  sudo apt-get install -qq libboost1.58-all-dev
elif [ $ubuntu_version == "1804" ];then
  sudo apt-get install -qq libboost1.65-all-dev
elif [ $ubuntu_version == "1910" ];then
    sudo apt-get install -qq libboost1.67-all-dev
    sudo apt-get install -qq g++
else
  exit 0
fi

sudo apt-get install -qq g++-5 cmake git freeglut3 freeglut3-dev libglpk-dev 
sudo apt-get install -qq libxmu-dev libxi-dev libqt4-dev libeigen3-dev libassimp-dev libflann-dev liburdfdom-tools libccd-dev libqhull-dev 
sudo apt-get install -qq python-dev python-opengl python-setuptools pypy python-tk
sudo apt-get install -qq xclip openctm-tools

echo "***********************************************************************"
echo "Installing Libraries for Python (Optional)"
echo "***********************************************************************"
pip install --user --upgrade pip
pip install --user matplotlib
pip install --user scipy
pip install --user cvxpy
pip install --user pdf2image
pip install --user openmesh
pip install --user trimesh

mkdir -p ${INSTALL_DIR}/libs
sudo cp scripts/converter* /usr/bin/

echo "***********************************************************************"
echo "Installing OMPL (Planning Library)"
echo "***********************************************************************"
cd ${INSTALL_DIR}/libs/
git clone git@github.com:aorthey/ompl.git
cd ompl
mkdir build
cd build/
cmake ..
make -j$(nproc)
sudo make install
# wget http://ompl.kavrakilab.org/install-ompl-ubuntu.sh
# chmod u+x install-ompl-ubuntu.sh
# ./install-ompl-ubuntu.sh --app

echo "***********************************************************************"
echo "Installing LEMON (Graph Library)"
echo "***********************************************************************"
cd ${INSTALL_DIR}/libs/
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
cd ${INSTALL_DIR}/libs/
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
echo "Installing MotionPlanningExplorerGUI"
echo "***********************************************************************"
cd ${INSTALL_DIR}
mkdir -p build
cd build
cmake ..
make -j$(nproc) planner_gui
./planner_gui ../data/experiments/02D_manipulator.xml
