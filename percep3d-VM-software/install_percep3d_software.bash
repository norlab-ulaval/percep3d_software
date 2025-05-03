#!/bin/bash -i
# =================================================================================================
# Percept3D course software install
#
# Maintainer: luc.coupal.1@ulaval.ca
#
# Script usage:
#   1. In the VM, execute the following line in a terminal
#       $ sudo apt-get update && sudo apt-get install --assume-yes git
#       $ cd /opt && sudo git clone https://github.com/norlab-ulaval/percep3d_software.git
#       $ cd percep3d_software/percep3d-VM-software && sudo bash install_percep3d_software.bash
#   2. logout current user and login with user `student` pass `percep3d`
#
# Note on unit test:
#    $ docker pull --platform linux/arm64 ubuntu:20.04
#    $ docker build --platform linux/arm64 -f Dockerfile.test -t percep3d-vm-software-tester-ubuntu:20.04 .
#    $ docker run -a --name iAmTestROSmelodic4vmContainer -t -i percep3d-vm-software-tester-ubuntu:20.04
#
# =================================================================================================
set -e # Note: we want the installer to always fail-fast (it wont affect the build system policy)


# ....Hardcoded environment variable...............................................................
ROS_PKG='desktop_full'
P3D_USER='student'
PASSWORD='percep3d'
PERCEPT_LIBRARIES_PATH="/opt/percep3d_libraries"

SETUP_SSH_DAEMON=false
SETUP_SSH_DAEMON=${SETUP_SSH_DAEMON:-true} # Skip ssh daemon setup if set to false


# ....Auto set ROS distro..........................................................................
echo -e "\nAuto set ROS distro\n"

sudo apt-get update \
  && sudo apt-get install --assume-yes \
      lsb-release \
  && sudo rm -rf /var/lib/apt/lists/*

# Retrieve ubuntu version number: DISTRIB_RELEASE
source /etc/lsb-release
if [[ ${DISTRIB_RELEASE} == '18.04' ]]; then
  ROS_DISTRO='melodic'
elif [[ ${DISTRIB_RELEASE} == '20.04' ]]; then
  ROS_DISTRO='noetic'
else
  echo "Ubuntu distro ${DISTRIB_RELEASE} not supported by the installer"
  exit 1
fi
echo "Ubuntu version is ${DISTRIB_RELEASE}, will install ROS distro ${ROS_DISTRO}"



# ... Add new user ................................................................................
echo -e "\nAdd new user\n"

P3D_USER_HOME="/home/${P3D_USER}"

# $ sudo useradd -s /path/to/shell -d /home/{dirname} -m -G {secondary-group} {username}
sudo useradd -s /bin/bash -d "${P3D_USER_HOME}" -m "${P3D_USER}" \
  && yes "${PASSWORD}" | passwd "${P3D_USER}"
# Add sudo group to P3D_USER
sudo usermod -a -G sudo "${P3D_USER}"
# Note: Add the 'video' groups to new user as it's required for GPU access.
# (not a problem on norlab-og but mandatory on Jetson device)
# Ref: https://forums.developer.nvidia.com/t/how-to-properly-create-new-users/68660/2


# .... Create required dir structure ..............................................................
echo -e "\nCreate required dir structure\n"

DS_ROS_ROOT="/opt/ros/${ROS_DISTRO}"
ROS_DEV_WORKSPACE="${P3D_USER_HOME}/catkin_ws"

mkdir -p "${DS_ROS_ROOT}"
mkdir -p "${ROS_DEV_WORKSPACE}/src"
mkdir -p "${PERCEPT_LIBRARIES_PATH}"
mkdir -p "${P3D_USER_HOME}/percep3d_data"


# . . Add archived files . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
echo -e "\nAdd archived files\n"

sudo apt-get update \
    && sudo apt-get install --assume-yes \
        apt-utils \
        zip gzip tar unzip \
    && sudo rm -rf /var/lib/apt/lists/*

# (CRITICAL) Don't execute `cd` before the folling lines
cp "./beginner_tutorials.zip" "${ROS_DEV_WORKSPACE}/src"
cp "./percep3d_mapping.zip" "${ROS_DEV_WORKSPACE}/src"

cd "${ROS_DEV_WORKSPACE}/src"
unzip beginner_tutorials.zip
unzip percep3d_mapping.zip
rm beginner_tutorials.zip
rm percep3d_mapping.zip




# ==== Install tools ==============================================================================
echo -e "\nInstall tools\n"

# skip GUI dialog by setting everything to default
export DEBIAN_FRONTEND=noninteractive

# ... install development utilities ...............................................................
sudo apt-get update \
    && apt-get upgrade --assume-yes \
    && sudo apt-get install --assume-yes \
        gnupg2 \
        g++ make cmake \
        build-essential \
        curl \
        wget \
        libusb-dev \
        ca-certificates \
        git \
        usbutils \
        vim \
        tree \
        bash-completion \
        net-tools \
    && sudo rm -rf /var/lib/apt/lists/*



# .... hardware acceleration in VM ................................................................
sudo apt-get update \
    && sudo apt-get install --assume-yes \
        mesa-utils \
    && sudo rm -rf /var/lib/apt/lists/*

( \
  echo "# Turn off hardware acceleration. Workaround for Mesa graphics drivers problem when running from a VM"; \
  echo "# ref: https://wiki.ros.org/rviz/Troubleshooting"; \
  echo "export LIBGL_ALWAYS_SOFTWARE=1"; \
)  >> ${P3D_USER_HOME}/.bashrc



# ===Service: ssh server===========================================================================
VAGRANT_PRIMARY_PORT=22

if [[ ${SETUP_SSH_DAEMON} == true ]]; then
  echo -e "\nInstall and configure ssh daemon\n"
  VM_SSH_SERVER_PORT=2222

  # install development utilities
  sudo apt-get update \
      && sudo apt-get install --assume-yes  \
          openssh-server \
      && sudo apt-get clean \
      && sudo rm -rf /var/lib/apt/lists/*

  # This will overright the vagrant box VAGRANT_PRIMARY_PORT
  ( \
      echo "LogLevel DEBUG2"; \
      echo "PermitRootLogin yes"; \
      echo "PasswordAuthentication yes"; \
      echo "Port ${VM_SSH_SERVER_PORT}"; \
    ) >> /etc/ssh/sshd_config \
    && mkdir -p /run/sshd

  sudo service ssh --full-restart

  echo -e "Check that ssh is running properly\n\n$(ps -aux | grep -e sshd -e USER)\n"
else
  echo -e "\nSkip ssh daemon install and configuration\n"
  VM_SSH_SERVER_PORT=$VAGRANT_PRIMARY_PORT
fi

# ==== Install percept3D libraries and dependencies ===============================================

# .... Dependencies ...............................................................................
echo -e "\nInstall percept3D libraries and dependencies\n"

if [[ ${ROS_DISTRO} == 'melodic' ]]; then
    sudo apt-get update \
        && sudo apt-get install --assume-yes \
            python-dev \
            python-opengl \
            python-numpy \
        && sudo rm -rf /var/lib/apt/lists/*;

    # Required dependencies for tutorial: Introduction to tf https://wiki.ros.org/tf/Tutorials/Introduction%20to%20tf
    # Work around to install pip in python2
    curl https://bootstrap.pypa.io/pip/2.7/get-pip.py --output get-pip.py
    sudo python2 get-pip.py
    python2 -m  pip install --no-cache-dir --verbose \
        pyyaml
else
    sudo apt-get update \
        && sudo apt-get install --assume-yes \
            python3-dev \
            python3-opengl \
            python3-numpy \
            python3-pip \
            python-is-python3 \
        && sudo rm -rf /var/lib/apt/lists/*;

    python3 -m pip install --upgrade pip
fi





## . . Install boost. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
## https://www.boost.org/doc/libs/1_79_0/more/getting_started/unix-variants.html
#sudo apt-get update \
#    && sudo apt-get install --assume-yes \
#        libboost-all-dev \
#    && sudo rm -rf /var/lib/apt/lists/*
#
## . . Install eigen . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
#sudo apt-get update \
#    && sudo apt-get install --assume-yes \
#        libeigen3-dev \
#    && sudo rm -rf /var/lib/apt/lists/*
#
## . . Install libnabo . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
#
## (!) ANN was not mentionned in doc
## ANN is a library written in C++, which supports data structures and algorithms for both exact and
## approximate nearest neighbor searching in arbitrarily high dimensions.
## https://www.cs.umd.edu/~mount/ANN/
#cd "${ROS_DEV_WORKSPACE}"
#wget https://www.cs.umd.edu/~mount/ANN/Files/1.1.2/ann_1.1.2.tar.gz
#tar xzf ann_1.1.2.tar.gz
#cd ann_1.1.2/
#make linux-g++
#sudo cp lib/libANN.a /usr/local/lib/
#sudo cp include/ANN/ANN.h /usr/local/include/
## shellcheck disable=SC2103
#cd ..
#
#
## (!) FLANN was not mentionned in doc
## Fast Library for Approximate Nearest Neighbors - development
## FLANN is a library for performing fast approximate nearest neighbor searches
## in high dimensional spaces.
## https://github.com/flann-lib/flann
#sudo apt-get update \
#    && sudo apt-get install --assume-yes \
#        libflann-dev \
#    && sudo rm -rf /var/lib/apt/lists/*
#
#
#cd "${PERCEPT_LIBRARIES_PATH}"
## https://github.com/ethz-asl/libnabo
#git clone https://github.com/ethz-asl/libnabo.git \
#    && cd libnabo \
#    && mkdir build && cd build \
#    && cmake -D CMAKE_BUILD_TYPE=RelWithDebInfo .. \
#    && make -j $(nproc) \
#    && make test \
#    && sudo make install
#
##    && git checkout 1.0.7 \
#
## ToDo:on task end >> next bloc ↓↓
##pwd && tree -L 3
#
## . . Install percept3D libraries. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
#cd "${PERCEPT_LIBRARIES_PATH}"
#
#sudo apt-get update \
#    && sudo apt-get install --assume-yes \
#        libyaml-cpp-dev \
#    && sudo rm -rf /var/lib/apt/lists/*
#
## https://github.com/ethz-asl/libpointmatcher/tree/master
#git clone https://github.com/ethz-asl/libpointmatcher.git \
#    && cd libpointmatcher \
#    && mkdir build && cd build \
#    && cmake -D CMAKE_BUILD_TYPE=RelWithDebInfo \
#            -D BUILD_TESTS=TRUE \
#             .. \
#    && make -j $(nproc) \
#    && sudo make install
#
##            -DCMAKE_INSTALL_PREFIX=/usr/local/ \
##    && git checkout 1.3.1 \
#
#cd "${PERCEPT_LIBRARIES_PATH}/libpointmatcher/build"
#utest/utest --path "${PERCEPT_LIBRARIES_PATH}/libpointmatcher/examples/data/"
#
#
#cd "${PERCEPT_LIBRARIES_PATH}"
#git clone https://github.com/norlab-ulaval/norlab_icp_mapper.git \
#    && mkdir -p norlab_icp_mapper/build && cd norlab_icp_mapper/build \
#    && cmake -DCMAKE_BUILD_TYPE=Release .. \
#    && make -j $(nproc) \
#    && sudo make install
#
#

# //// REFACTORING inprogress ↓↓ /////////////////////////////////////////////////////////////<--//

echo -e "\nInstall libpointmatcher\n"

cd "${PERCEPT_LIBRARIES_PATH}"
git clone --recurse-submodules https://github.com/ethz-asl/libpointmatcher.git
cd libpointmatcher


#wget https://bootstrap.pypa.io/get-pip.py

# (CRITICAL) ToDo: validate >> this line ↓ (the yes pipe)
yes 1 | bash libpointmatcher_dependencies_installer.bash

export APPEND_TO_CMAKE_FLAG=( "-D CMAKE_INSTALL_PREFIX=${PERCEPT_LIBRARIES_PATH:?err}" )
source lpm_install_libpointmatcher_ubuntu.bash \
      --compile-test \
      --cmake-build-type RelWithDebInfo

# (CRITICAL) ToDo: on task end >> unmute next bloc ↓↓
#cd "${PERCEPT_LIBRARIES_PATH}/libpointmatcher/build_system/ubuntu/"
#bash lpm_execute_libpointmatcher_unittest.bash

echo -e "\nInstall norlab_icp_mapper\n"
cd "${PERCEPT_LIBRARIES_PATH}"
git clone https://github.com/norlab-ulaval/norlab_icp_mapper.git \
    && mkdir -p norlab_icp_mapper/build && cd norlab_icp_mapper/build \
    && cmake -D CMAKE_BUILD_TYPE=Release \
             -D CMAKE_INSTALL_PREFIX=${PERCEPT_LIBRARIES_PATH:?err} \
             .. \
    && make -j $(nproc) \
    && sudo make install

# //////////////////////////////////////////////////////////////// REFACTORING inprogress ////<--//

# === ROS =========================================================================================
echo -e "\nInstall ROS1\n"

# ... register the ROS package source .............................................................
# Setup sources.lst
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

## Quick hack for installing ROS melodic on Ubuntu 20.04 Focal
#sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu bionic main" > /etc/apt/sources.list.d/ros-latest.list'

# Setup keys
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -


# ... Install ROS,  ...............................................................................

# Credit for the next two RUN step: NVIDIA-AI-IOT/ros2_jetson
#    https://github.com/NVIDIA-AI-IOT/ros2_jetson/blob/main/docker/DockerFile.l4tbase.ros.noetic

mkdir -p "${ROS_DEV_WORKSPACE}/src/"
cd "${ROS_DEV_WORKSPACE}/src/"

if [[ ${ROS_DISTRO} == 'melodic' ]]; then
    apt-get update \
        && apt-get install --assume-yes \
            ros-${ROS_DISTRO}-$(echo "${ROS_PKG}" | tr '_' '-') \
            python-rosdep \
            python-rosinstall-generator \
            python-vcstool \
            python-wstool \
            python-rosinstall \
        && sudo rosdep init;
else
    apt-get update \
        && apt-get install --assume-yes \
            ros-${ROS_DISTRO}-$(echo "${ROS_PKG}" | tr '_' '-') \
            python3-rosdep \
            python3-rosinstall-generator \
            python3-vcstool \
            python3-wstool \
            python3-rosinstall \
        && sudo rosdep init;
fi

echo -e "\nInstall ROS1: rosdep update & install\n"

apt-get update --fix-missing
rosdep update
sudo rosdep fix-permissions

cd "${ROS_DEV_WORKSPACE}"
sudo apt-get update \
    && rosdep install --from-paths ./src \
      --ignore-packages-from-source \
      --rosdistro=${ROS_DISTRO} \
      --include-eol-distros \
      -y


echo -e "\nInstall ROS1: execute catkin_make\n"

source "${DS_ROS_ROOT}/setup.bash"
catkin_make
source "${ROS_DEV_WORKSPACE}/devel/setup.bash"

echo "source ${DS_ROS_ROOT}/setup.bash" >> ~/.bashrc
echo "source ${ROS_DEV_WORKSPACE}/devel/setup.bash" >> ~/.bashrc
echo "source ${DS_ROS_ROOT}/setup.bash" >> "${P3D_USER_HOME}/.bashrc"
echo "source ${ROS_DEV_WORKSPACE}/devel/setup.bash" >> "${P3D_USER_HOME}/.bashrc"
# Make sure your workspace is properly overlayed by the setup script by checking
# the ROS_PACKAGE_PATH environment variable. It should include the directory you're in:
#   $ echo $ROS_PACKAGE_PATH
#   > /home/youruser/ros_catkin_ws/src:/opt/ros/melodic/share


## . . Pull required repository. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
echo -e "\nPull ROS required repository\n"

cd "${ROS_DEV_WORKSPACE}/src/"
# Note: --branch melodic should work for both ROS1 distro (noetic and melodic)
git clone --branch melodic https://github.com/norlab-ulaval/norlab_icp_mapper_ros.git
git clone --branch melodic https://github.com/norlab-ulaval/libpointmatcher_ros.git
git clone https://github.com/norlab-ulaval/percep3d_turtle_exercises.git


# . . Install ROS & build catkin workspace. . . . . . . . . . . . . . . . . . . . . . . . . . . . .
echo -e "\nInstall ROS required repository\n"
cd "${ROS_DEV_WORKSPACE}"

# (Priority) ToDo:validate >> next bloc
#export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:${PERCEPT_LIBRARIES_PATH}/norlab_icp_mapper

sudo apt-get update
source "${DS_ROS_ROOT}/setup.bash"
#catkin_make_isolated
catkin_make
source "${ROS_DEV_WORKSPACE}/devel/setup.bash"


# Required dependencies for tutorial: Introduction to tf https://wiki.ros.org/tf/Tutorials/Introduction%20to%20tf
# Note: tf is deprecated in favor of tf2 ››› Install tf2 for tutorial in exo module 2.4
sudo apt-get update \
    && sudo apt-get install --assume-yes \
          ros-${ROS_DISTRO}-turtle-tf2 \
          ros-${ROS_DISTRO}-tf2-tools \
          ros-${ROS_DISTRO}-tf \
    && sudo rm -rf /var/lib/apt/lists/*


# .... install Paraview ...........................................................................
echo -e "\nInstall Paraview\n"

sudo apt-get update \
    && sudo apt-get install --assume-yes \
        paraview \
    && sudo rm -rf /var/lib/apt/lists/*


# ....Fetch ros bag husky_short_demo.bag...........................................................
echo -e "\nFetch ros bag husky_short_demo.bag\n"

cd "${P3D_USER_HOME}/percep3d_data"
wget -O husky_short_demo.zip "http://norlab.s3.valeria.science/percep3d/husky_short_demo.zip?AWSAccessKeyId=XMBLP3A0338XN5LASKV2&Expires=2319980812&Signature=n5HiUTunG7tcTINJovxH%2FtnGbM4%3D"
unzip husky_short_demo.zip
rm husky_short_demo.zip


# ==== Final step =================================================================================
echo -e "\nFinalize percep3d-software-install\n"

# Make sure that you have your environment properly setup. A good way to check is to ensure that
# environment variables like ROS_ROOT and ROS_PACKAGE_PATH are set:
#   $ printenv | grep ROS
printenv | grep ROS

cd "${ROS_DEV_WORKSPACE}"

# Change directory ownership
sudo chown -R student:student "${P3D_USER_HOME}/percep3d_data"
sudo chown -R student:student "${ROS_DEV_WORKSPACE}"

echo -e "To connect remotely to the container:
    $ ssh -p ${VM_SSH_SERVER_PORT} ${P3D_USER}@$(hostname -I | awk '{print $1}')
    $ sftp -P ${VM_SSH_SERVER_PORT} openssh-$(hostname -I | awk '{print $1}')
    $ scp -P ${VM_SSH_SERVER_PORT} /path/to/foo ${P3D_USER}@$(hostname -I | awk '{print $1}'):/dest/
"

# /////////////////////////////////////////////////////////// Percept3D course software install ///
