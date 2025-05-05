#!/bin/bash -i
# =================================================================================================
# Percep3D course software install (ROS1 version)
#
# Maintainer: luc.coupal.1@ulaval.ca
#
# Script usage:
#   1. In the VM, execute the following line in a terminal
#       $ sudo apt-get update && sudo apt-get install --assume-yes git
#       $ cd /opt
#       $ sudo git clone --recurse-submodules https://github.com/norlab-ulaval/percep3d_software.git
#       $ cd percep3d_software/vm_software_install_ros1 && sudo bash install_percep3d_software_ros1.bash
#   2. logout current user and login with user `student` pass `percep3d`
#
# Note on unit test:
#    $ docker pull --platform linux/arm64 ubuntu:20.04
#    $ docker build --platform linux/arm64 -f Dockerfile.test -t percep3d-vm-software-tester-ros1-ubuntu:20.04 .
#    $ docker run -a --name iAmTestROSmelodic4vmContainer -t -i percep3d-vm-software-tester-ros1-ubuntu:20.04
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
VAGRANT_SSH_PORT=22

SHOW_SPLASH_IDU="${SHOW_SPLASH_IDU:-false}"

P3D_USER_HOME="/home/${P3D_USER}"
P3D_ROS_DEV_WORKSPACE="${P3D_USER_HOME}/catkin_ws"

# ....Project root logic...........................................................................
TMP_CWD=$(pwd)
P3DS_PATH=$(git rev-parse --show-toplevel)
cd "${P3DS_PATH}" || exit 1


# ....Helper function..............................................................................
N2ST_PATH=${N2ST_PATH:-"${P3DS_PATH}/utilities/norlab-shell-script-tools"}

cd "${N2ST_PATH}" || exit 1
source import_norlab_shell_script_tools_lib.bash

# ====Begin========================================================================================

# ....Setup timezone and localization..............................................................
# change the locale from POSIX to UTF-8
# ToDo: assessment >> next bloc ↓↓ as a work arround for vagrant splash console print problem
sudo apt-get update && \
  sudo apt-get install --assume-yes --no-install-recommends \
      locales \
      lsb-release \
  && sudo rm -rf /var/lib/apt/lists/* \
  && locale-gen en_US en_US.UTF-8 \
  && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

if [[ "${SHOW_SPLASH_IDU}" == 'true' ]]; then
  n2st::norlab_splash "Percep3D course software install" "https://github.com/norlab-ulaval/percep3d_software"
fi

n2st::print_formated_script_header "install_percep3d_software_ros1.bash"


# ....Auto set ROS distro..........................................................................
n2st::print_formated_script_header "Auto set ROS distro" "."
#n2st::print_msg "Auto set ROS distro"

# Retrieve ubuntu version number: DISTRIB_RELEASE
source /etc/lsb-release
if [[ ${DISTRIB_RELEASE} == '18.04' ]]; then
  ROS_DISTRO='melodic'
elif [[ ${DISTRIB_RELEASE} == '20.04' ]]; then
  ROS_DISTRO='noetic'
else
  n2st::print_msg_error_and_exit "Ubuntu distro ${DISTRIB_RELEASE} not supported by the installer"
fi
P3D_ROS_ROOT="/opt/ros/${ROS_DISTRO}"
echo "Ubuntu version is ${DISTRIB_RELEASE}, will install ROS1 distro ${ROS_DISTRO} at ${P3D_ROS_ROOT}"


# //// MUTE FROM HERE ↓↓ /////////////////////////////////////////////////////////////////////<--//

## ... Add new user ................................................................................
#n2st::print_formated_script_header "Add new user" "."
#
#
## $ sudo useradd -s /path/to/shell -d /home/{dirname} -m -G {secondary-group} {username}
#sudo useradd -s /bin/bash -d "${P3D_USER_HOME}" -m "${P3D_USER}" \
#  && yes "${PASSWORD}" | sudo passwd "${P3D_USER}"
## Add sudo group to P3D_USER
#sudo usermod -a -G sudo "${P3D_USER}"
## Note: Add the 'video' groups to new user as it's required for GPU access.
## (not a problem on norlab-og but mandatory on Jetson device)
## Ref: https://forums.developer.nvidia.com/t/how-to-properly-create-new-users/68660/2
#
#
## .... Create required dir structure ..............................................................
#n2st::print_formated_script_header "Create required dir structure" "."
#
#
#sudo mkdir -p "${P3D_ROS_ROOT}"
#sudo mkdir -p "${P3D_ROS_DEV_WORKSPACE}/src"
#sudo mkdir -p "${PERCEPT_LIBRARIES_PATH}"
#sudo mkdir -p "${P3D_USER_HOME}/percep3d_data"
#
#
## . . Add archived files . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
#n2st::print_formated_script_header "Add archived files" "."
#
#sudo apt-get update \
#    && sudo apt-get install --assume-yes \
#        apt-utils \
#        zip gzip tar unzip \
#    && sudo rm -rf /var/lib/apt/lists/*
#
#cd "${P3DS_PATH}/vm_software_install_ros1" || exit 1
#
## (CRITICAL) Don't execute `cd` before the folling lines
#sudo cp "./beginner_tutorials.zip" "${P3D_ROS_DEV_WORKSPACE}/src"
#sudo cp "./percep3d_mapping.zip" "${P3D_ROS_DEV_WORKSPACE}/src"
#
#cd "${P3D_ROS_DEV_WORKSPACE}/src"
#sudo unzip beginner_tutorials.zip
#sudo unzip percep3d_mapping.zip
#sudo rm beginner_tutorials.zip
#sudo rm percep3d_mapping.zip
#
#
#
#
## ==== Install tools ==============================================================================
#n2st::print_formated_script_header "Install tools" "."
#
## skip GUI dialog by setting everything to default
#export DEBIAN_FRONTEND=noninteractive
#
## ... install development utilities ...............................................................
#sudo apt-get update \
#    && sudo apt-get install --assume-yes \
#        gnupg2 \
#        g++ make cmake \
#        build-essential \
#        curl \
#        wget \
#        libusb-dev \
#        ca-certificates \
#        git \
#        usbutils \
#        vim \
#        tree \
#        bash-completion \
#        net-tools \
#    && sudo rm -rf /var/lib/apt/lists/*
#
#
#
## .... hardware acceleration in VM ................................................................
#sudo apt-get update \
#    && sudo apt-get install --assume-yes \
#        mesa-utils \
#    && sudo rm -rf /var/lib/apt/lists/*
#
#( \
#  echo "# Turn off hardware acceleration. Workaround for Mesa graphics drivers problem when running from a VM"; \
#  echo "# ref: https://wiki.ros.org/rviz/Troubleshooting"; \
#  echo "export LIBGL_ALWAYS_SOFTWARE=1"; \
#)  | sudo tee --append ${P3D_USER_HOME}/.bashrc
#
#
#
## ===Service: ssh server===========================================================================
#
#
#n2st::print_formated_script_header "ssh daemon setup" "."
#
#if [[ ${SETUP_SSH_DAEMON} == true ]]; then
#  n2st::print_msg "Install and configure ssh daemon"
#  VM_SSH_SERVER_PORT=2222
#
#  # install development utilities
#  sudo apt-get update \
#      && sudo apt-get install --assume-yes  \
#          openssh-server \
#      && sudo apt-get clean \
#      && sudo rm -rf /var/lib/apt/lists/*
#
#  # This will overright the vagrant box VAGRANT_SSH_PORT
#  ( \
#      echo "LogLevel DEBUG2"; \
#      echo "PermitRootLogin yes"; \
#      echo "PasswordAuthentication yes"; \
#      echo "Port ${VM_SSH_SERVER_PORT}"; \
#    ) >> /etc/ssh/sshd_config \
#    && mkdir -p /run/sshd
#
#  sudo service ssh --full-restart
#
#  print_msg "Check that ssh is running properly\n\n$(ps -aux | grep -e sshd -e USER)\n"
#else
#  n2st::print_msg "Skip ssh daemon install and configuration"
#  VM_SSH_SERVER_PORT=$VAGRANT_SSH_PORT
#fi
#
## ==== Install percep3D libraries and dependencies ===============================================
#
## .... Dependencies ...............................................................................
#n2st::print_formated_script_header "Install percep3D libraries and dependencies" "."
#
#if [[ ${ROS_DISTRO} == 'melodic' ]]; then
#    sudo apt-get update \
#        && sudo apt-get install --assume-yes \
#            python-dev \
#            python-opengl \
#            python-numpy \
#        && sudo rm -rf /var/lib/apt/lists/*;
#
#    # Required dependencies for tutorial: Introduction to tf https://wiki.ros.org/tf/Tutorials/Introduction%20to%20tf
#    # Work around to install pip in python2
#    curl https://bootstrap.pypa.io/pip/2.7/get-pip.py --output get-pip.py
#    sudo python2 get-pip.py
#    python2 -m  pip install --no-cache-dir --verbose \
#        pyyaml
#else
#    sudo apt-get update \
#        && sudo apt-get install --assume-yes \
#            python3-dev \
#            python3-opengl \
#            python3-numpy \
#            python3-pip \
#            python-is-python3 \
#        && sudo rm -rf /var/lib/apt/lists/*;
#
#    python3 -m pip install --upgrade pip
#fi
#
#
#
#
#
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
#n2st::print_formated_script_header "Install libnabo dependencies" "."
#
## (!) ANN was not mentionned in doc
## ANN is a library written in C++, which supports data structures and algorithms for both exact and
## approximate nearest neighbor searching in arbitrarily high dimensions.
## https://www.cs.umd.edu/~mount/ANN/
#cd "${P3D_ROS_DEV_WORKSPACE}"
#sudo wget https://www.cs.umd.edu/~mount/ANN/Files/1.1.2/ann_1.1.2.tar.gz
#sudo tar xzf ann_1.1.2.tar.gz
#cd ann_1.1.2/
#sudo make linux-g++
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
#n2st::print_formated_script_header "Install libnabo" "."
#cd "${PERCEPT_LIBRARIES_PATH}"
#sudo git clone https://github.com/norlab-ulaval/libnabo.git \
#    && cd libnabo \
#    && sudo git checkout c925c4709a383b702d547993df8842a42bbeb230 \
#    && sudo mkdir build && cd build \
#    && sudo cmake -D CMAKE_BUILD_TYPE=RelWithDebInfo \
#             -D CMAKE_INSTALL_PREFIX=${PERCEPT_LIBRARIES_PATH:?err} \
#             .. \
#    && sudo make -j $(nproc) \
#    && sudo make install \
#    || exit 1
#
##    && make test \
##    && git checkout 1.0.7 \
##    && git checkout 1.1.0 \
#
## Note libnabo tags:
##  - Checkout 1.0.7 -> Feb 11 2019
##  - Checkout 1.1.0 -> Jan 31 2024
#
##
## . . Install percep3D libraries. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
#n2st::print_formated_script_header "Install libpointmatcher" "."
#
#sudo apt-get update \
#    && sudo apt-get install --assume-yes \
#        libyaml-cpp-dev \
#    && sudo rm -rf /var/lib/apt/lists/*
#
#cd "${PERCEPT_LIBRARIES_PATH}"
#
## https://github.com/ethz-asl/libpointmatcher
#sudo git clone --recurse-submodules https://github.com/norlab-ulaval/libpointmatcher.git \
#    && cd libpointmatcher \
#    && sudo git checkout 1.4.0 \
#    && sudo mkdir build && cd build \
#    && sudo cmake -D CMAKE_BUILD_TYPE=RelWithDebInfo \
#             -D CMAKE_INSTALL_PREFIX=${PERCEPT_LIBRARIES_PATH:?err} \
#             -D BUILD_TESTS=TRUE \
#             .. \
#    && sudo make -j $(nproc) \
#    && sudo make install \
#    || exit 1
#
##    && git checkout d9c14b290fc59f59c497979a03c820a91dcbe96a \
##    && git checkout 1.3.1 \
#
## Note libpointmatcher tags:
##  - Checkout 1.3.1 -> Mar 4, 2019
##  - Checkout 1.4.0 -> Dec 29, 2023
#
### (CRITICAL) ToDo: on task end >> UN-mute next bloc ↓↓
##cd "${PERCEPT_LIBRARIES_PATH}/libpointmatcher/build" || exit 1
##utest/utest --path "${PERCEPT_LIBRARIES_PATH}/libpointmatcher/examples/data/"
#
#
#n2st::print_formated_script_header "Install norlab_icp_mapper" "."
#
#cd "${PERCEPT_LIBRARIES_PATH}"
#sudo git clone https://github.com/norlab-ulaval/norlab_icp_mapper.git \
#    && cd norlab_icp_mapper \
#    && sudo git checkout 7d66288e01f2e4f820edbeb97c5ffe420cc5b230 \
#    && sudo mkdir -p build && cd build \
#    && sudo cmake -D CMAKE_BUILD_TYPE=Release \
#             -D CMAKE_INSTALL_PREFIX=${PERCEPT_LIBRARIES_PATH:?err} \
#             .. \
#    && sudo make -j $(nproc) \
#    && sudo make install \
#    || exit 1
#
##    && git checkout 67fc0f892e208d1d5ec973062944d6e88b663baf \
#
## Note norlab_icp_mapper tags:
##  - Checkout 67fc0f892e208d1d5ec973062944d6e88b663baf -> June 3, 2022
##  - Checkout 7d66288e01f2e4f820edbeb97c5ffe420cc5b230 -> Mar 22, 2022

# //// WE ARE HERE ↓↓ ////////////////////////////////////////////////////////////////////////<--//
# === ROS =========================================================================================
n2st::print_formated_script_header "Install ROS1" "."

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

sudo mkdir -p "${P3D_ROS_DEV_WORKSPACE}/src/"
cd "${P3D_ROS_DEV_WORKSPACE}/src/"

if [[ ${ROS_DISTRO} == 'melodic' ]]; then
    sudo apt-get update \
        && sudo apt-get install --assume-yes \
            ros-"${ROS_DISTRO}"-$(echo "${ROS_PKG}" | tr '_' '-') \
            python-rosdep \
            python-rosinstall-generator \
            python-vcstool \
            python-wstool \
            python-rosinstall \
        && sudo rosdep init \
        || exit 1
else
    sudo apt-get update \
        && sudo apt-get install --assume-yes \
            ros-"${ROS_DISTRO}"-$(echo "${ROS_PKG}" | tr '_' '-') \
            python3-rosdep \
            python3-rosinstall-generator \
            python3-vcstool \
            python3-wstool \
            python3-rosinstall \
        && sudo rosdep init \
        || exit 1
fi

n2st::print_formated_script_header "Install ROS1: rosdep update & install" "."

sudo apt-get update --fix-missing
sudo apt-get install --assume-yes \
     python3-catkin-pkg \
     python3-catkin-pkg-modules

rosdep update
#sudo rosdep fix-permissions

cd "${P3D_ROS_DEV_WORKSPACE}"
rosdep install \
    --ignore-packages-from-source \
    --from-paths ./src \
    --rosdistro"=${ROS_DISTRO}" \
    --include-eol-distros \
    -y \
    || exit 1

n2st::print_formated_script_header "Install ROS1: setup catkin workspace" "."

source "${P3D_ROS_ROOT}/setup.bash"
catkin_make
source "${P3D_ROS_DEV_WORKSPACE}/devel/setup.bash"

echo "source ${P3D_ROS_ROOT}/setup.bash" >> ~/.bashrc
echo "source ${P3D_ROS_DEV_WORKSPACE}/devel/setup.bash" >> ~/.bashrc
echo "source ${P3D_ROS_ROOT}/setup.bash" >> "${P3D_USER_HOME}/.bashrc"
echo "source ${P3D_ROS_DEV_WORKSPACE}/devel/setup.bash" >> "${P3D_USER_HOME}/.bashrc"

# ....ROS1 install sanity check....................................................................
n2st::print_formated_script_header "ROS1 install sanity check" "."


echo "sourcing ${P3D_ROS_ROOT}/setup.bash" \
  && source "${P3D_ROS_ROOT}"/setup.bash \
  && echo "sourcing ${P3D_ROS_DEV_WORKSPACE}/devel/setup.bash" \
  && source "${P3D_ROS_DEV_WORKSPACE}"/devel/setup.bash \
  && echo ROS_VERSION="${ROS_VERSION:?'Build argument needs to be set and non-empty.'}" \
  && echo ROS_PYTHON_VERSION="${ROS_PYTHON_VERSION:?'Build argument needs to be set and non-empty.'}" \
  && echo ROS_DISTRO="${ROS_DISTRO:?'Build argument needs to be set and non-empty.'}" \
  && echo PATH="${PATH}" \
  && echo PYTHONPATH="${PYTHONPATH:?'Build argument needs to be set and non-empty.'}" \
  && echo CMAKE_PREFIX_PATH="${CMAKE_PREFIX_PATH:?'Build argument needs to be set and non-empty.'}" \
  && python -c "import rospy" \
  && [[ "$(rosversion --distro)" == "${ROS_DISTRO}" ]] \
  || exit 1

echo "Check workspace directory installation"  \
    && [[ -d ${P3D_ROS_DEV_WORKSPACE}/src ]] \
    && [[ -d ${P3D_ROS_DEV_WORKSPACE}/devel ]] \
    || exit 1


## . . Pull required repository. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
#n2st::print_formated_script_header "Pull ROS required repository" "."
#cd "${P3D_ROS_DEV_WORKSPACE}/src/"
#
## ///////////////////////////////////////////////////////////////////// MUTE TO HERE ↑↑ //////<--//
#
### (Priority) ToDo DEV: on task end >> mute next bloc ↓↓
##DEV_TO_BASHRC=$(cat <<'EOF'
##export P3D_USER=student
##export P3D_USER_HOME=/home/${P3D_USER}
##export PERCEPT_LIBRARIES_PATH=/opt/percep3d_libraries
##export P3D_ROS_ROOT=/opt/ros/${ROS_DISTRO}
##export P3D_ROS_DEV_WORKSPACE=${P3D_USER_HOME}/catkin_ws
##EOF
##)
##( \
##  echo "${DEV_TO_BASHRC[@]}"; \
##  echo ; \
##  echo "source ${P3D_ROS_ROOT}/setup.bash"; \
##  echo "source ${P3D_ROS_DEV_WORKSPACE}/devel/setup.bash"; \
##) >> /home/vagrant/.bashrc
##( \
##  echo "${DEV_TO_BASHRC[@]}"; \
##) >> /home/student/.bashrc
#
### //// REFACTORING inprogress ↓↓ ////////////////////////////////////////////////////////////<--//
### Note: --branch melodic should work for both ROS1 distro (noetic and melodic)
##cd "${P3D_ROS_DEV_WORKSPACE}/src/"
##git clone https://github.com/norlab-ulaval/libpointmatcher_ros.git
##cd libpointmatcher_ros
##git checkout 0a9e60e694adb9868a12c31808071eba8f0f2ec9
### Checkout 0a9e60e694adb9868a12c31808071eba8f0f2ec9 -> humble branch (default) Apr 19 2022
##
##
##cd "${P3D_ROS_DEV_WORKSPACE}/src/"
##git clone https://github.com/norlab-ulaval/norlab_icp_mapper_ros.git
##cd norlab_icp_mapper_ros
##git checkout 8aca7103af013bf13ddd0749e7f4512f9f9c6013
### Checkout 8aca7103af013bf13ddd0749e7f4512f9f9c6013 -> ros2 branch (default) Mar 23, 2022
##
##
##cd "${P3D_ROS_DEV_WORKSPACE}/src/"
##git clone https://github.com/norlab-ulaval/percep3d_turtle_exercises.git
##
##
##
##
### . . Install ROS & build catkin workspace. . . . . . . . . . . . . . . . . . . . . . . . . . . . .
##n2st::print_formated_script_header "Install ROS required repository" "."
##cd "${P3D_ROS_DEV_WORKSPACE}"
##
### (Priority) ToDo:validate >> next bloc
###export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:${PERCEPT_LIBRARIES_PATH}/norlab_icp_mapper
##
##apt-get update --fix-missing \
##    && apt-get install --assume-yes \
##        ros-${ROS_DISTRO}-nav-msgs \
##        ros-"${ROS_DISTRO}"-sensor-msgs
##
###sudo apt-get update
##source "${P3D_ROS_ROOT}/setup.bash"
##source "${P3D_ROS_DEV_WORKSPACE}/devel/setup.bash"
##
##apt-get update --fix-missing \
##    && rosdep update --rosdistro "${ROS_DISTRO}" \
##                     --include-eol-distros \
##    && rosdep fix-permissions \
##    && rosdep install  \
##            --ignore-packages-from-source \
##            --from-path ./src  \
##            --rosdistro "${ROS_DISTRO}" \
##            -y
##
###export CMAKE_PREFIX_PATH="${CMAKE_PREFIX_PATH}":"${PERCEPT_LIBRARIES_PATH:?err}/libpointmatcher":"${PERCEPT_LIBRARIES_PATH:?err}/norlab_icp_mapper"
###catkin_make --cmake-args -DCMAKE_PREFIX_PATH="${CMAKE_PREFIX_PATH}"
##catkin_make
##
#### Required dependencies for tutorial: Introduction to tf https://wiki.ros.org/tf/Tutorials/Introduction%20to%20tf
#### Note: tf is deprecated in favor of tf2 ››› Install tf2 for tutorial in exo module 2.4
###sudo apt-get update \
###    && sudo apt-get install --assume-yes \
###          ros-"${ROS_DISTRO}"-turtle-tf2 \
###          ros-"${ROS_DISTRO}"-tf2-tools \
###          ros-"${ROS_DISTRO}"-tf \
###    && sudo rm -rf /var/lib/apt/lists/*
###
###
#### .... install Paraview ...........................................................................
###n2st::print_formated_script_header "Install Paraview" "."
###
###sudo apt-get update \
###    && sudo apt-get install --assume-yes \
###        paraview \
###    && sudo rm -rf /var/lib/apt/lists/*
###
###
#### ....Fetch ros bag husky_short_demo.bag...........................................................
###n2st::print_formated_script_header "Fetch ros bag husky_short_demo.bag" "."
###
###cd "${P3D_USER_HOME}/percep3d_data"
###wget -O husky_short_demo.zip "http://norlab.s3.valeria.science/percep3d/husky_short_demo.zip?AWSAccessKeyId=XMBLP3A0338XN5LASKV2&Expires=2319980812&Signature=n5HiUTunG7tcTINJovxH%2FtnGbM4%3D"
###unzip husky_short_demo.zip
###rm husky_short_demo.zip
###
###
#### ==== Final step =================================================================================
###n2st::print_formated_script_header "Finalize percep3d-software-install" "."
###
#### Make sure that you have your environment properly setup. A good way to check is to ensure that
#### environment variables like ROS_ROOT and ROS_PACKAGE_PATH are set:
####   $ printenv | grep ROS
###printenv | grep ROS
###
###cd "${P3D_ROS_DEV_WORKSPACE}"
###
#### Change directory ownership
###sudo chown -R student:student "${P3D_USER_HOME}/percep3d_data"
###sudo chown -R student:student "${P3D_ROS_DEV_WORKSPACE}"
###
###n2st::print_msg_done "To connect remotely to the container:
###    $ ssh -p ${VM_SSH_SERVER_PORT} ${P3D_USER}@$(hostname -I | awk '{print $1}')
###    $ sftp -P ${VM_SSH_SERVER_PORT} openssh-$(hostname -I | awk '{print $1}')
###    $ scp -P ${VM_SSH_SERVER_PORT} /path/to/foo ${P3D_USER}@$(hostname -I | awk '{print $1}'):/dest/
###"
###
###n2st::print_formated_script_footer "install_percep3d_software_ros1.bash"
#### ====Teardown=====================================================================================
###cd "${TMP_CWD}" || exit
###
