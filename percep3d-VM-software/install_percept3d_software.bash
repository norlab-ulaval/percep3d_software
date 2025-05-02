#!/bin/bash -i

# /// Percept3D course software install ///////////////////////////////////////////////////////////
#
# Maintainer: luc.coupal.1@ulaval.ca
#
# Script usage:
#   1. In the VM, execute the following line in a terminal
#       $ cd /opt
#       $ sudo apt-get update && sudo apt-get install --assume-yes git
#       $ sudo git clone https://github.com/norlab-ulaval/percep3d_software.git
#       $ cd percep3d_software/percep3d-VM-software
#       $ sudo bash install_percept3d_software.bash
#   2. logout current user and login with user `student` pass `percept3d`
#
#   (!) Be advise that VM root password as also been change to `percept3d`
#
# Note on unit test:
#    $ docker pull --platform linux/arm64 ubuntu:20.04
#    $ docker build --platform linux/arm64 -f Dockerfile.test -t percep3d-vm-software-tester-ubuntu:20.04 .
#    $ docker run -a --name iAmTestROSmelodic4vmContainer -t -i percep3d-vm-software-tester-ubuntu:20.04
#
ROS_PKG='desktop_full'
ROS_DISTRO='melodic'
#ROS_DISTRO='noetic'
DS_ROS_ROOT="/opt/ros/${ROS_DISTRO}"


# ... Add new user ................................................................................
P3D_USER='student'
PASSWORD='percept3d'
P3D_USER_HOME="/home/${P3D_USER}"

# $ sudo useradd -s /path/to/shell -d /home/{dirname} -m -G {secondary-group} {username}
sudo useradd -s /bin/bash -d "${P3D_USER_HOME}" -m "${P3D_USER}" \
  && yes "${PASSWORD}" | passwd "${P3D_USER}"
# Add sudo group to P3D_USER
sudo usermod -a -G sudo "${P3D_USER}"
# Note: Add the 'video' groups to new user as it's required for GPU access.
# (not a problem on norlab-og but mandatory on Jetson device)
# Ref: https://forums.developer.nvidia.com/t/how-to-properly-create-new-users/68660/2

## ... root config ................................................................................
## (CRITICAL) ToDo:validate >> next bloc ↓↓
##  - Related to readme point › 3. ★ Be advise that VM root password has also been change to `percept3d`
##  - Configuring root is relevant for the Docker container version, but not sure it is for the shell script version.
##     Could potentially cause problem if someone execute the script by mistake in the host machine.
##  - Note on syntax › user:newpassword
#sudo echo "root:"${PASSWORD}"" | chpasswd


# .... Create required dir structure ..............................................................
PERCEPT_LIBRARIES="/opt/percep3d_libraries"
ROS_DEV_WORKSPACE="${P3D_USER_HOME}/catkin_ws"

mkdir -p "${DS_ROS_ROOT}"
mkdir -p "${ROS_DEV_WORKSPACE}/src"
mkdir -p "${PERCEPT_LIBRARIES}"
mkdir -p "${P3D_USER_HOME}/percep3d_data"


# . . Add archived files . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
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


# Fetch ros bag `husky_short_demo.bag`
cd "${P3D_USER_HOME}/percep3d_data"
wget -O husky_short_demo.zip "http://norlab.s3.valeria.science/percep3d/husky_short_demo.zip?AWSAccessKeyId=XMBLP3A0338XN5LASKV2&Expires=2319980812&Signature=n5HiUTunG7tcTINJovxH%2FtnGbM4%3D"
unzip husky_short_demo.zip
rm husky_short_demo.zip


# ==== Install tools ==============================================================================
cd "${ROS_DEV_WORKSPACE}"

# skip GUI dialog by setting everything to default
export DEBIAN_FRONTEND=noninteractive

# ... install development utilities ...............................................................
sudo apt-get update \
    && sudo apt-get install --assume-yes \
        lsb-release \
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

# install development utilities
sudo apt-get update \
    && sudo apt-get install --assume-yes  \
        openssh-server \
    && sudo apt-get clean \
    && sudo rm -rf /var/lib/apt/lists/*


# ...Setup ssh server..............................................................................
# Note: check that ssh is running `ps -aux | grep ssh`

# ssh port, remaped from default 22 to 2222
VM_SSH_SERVER_PORT=2222

# Inspired from https://austinmorlan.com/posts/docker_clion_development/
( \
    echo "LogLevel DEBUG2"; \
    echo "PermitRootLogin yes"; \
    echo "PasswordAuthentication yes"; \
    echo "Port ${VM_SSH_SERVER_PORT}"; \
  ) >> /etc/ssh/sshd_config \
  && mkdir /run/sshd

sudo service ssh --full-restart


# ==== Install percept3D libraries and dependencies ===============================================

# .... Dependencies ...............................................................................

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





# . . Install boost. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
# https://www.boost.org/doc/libs/1_79_0/more/getting_started/unix-variants.html
sudo apt-get update \
    && sudo apt-get install --assume-yes \
        libboost-all-dev \
    && sudo rm -rf /var/lib/apt/lists/*

# . . Install eigen . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
sudo apt-get update \
    && sudo apt-get install --assume-yes \
        libeigen3-dev \
    && sudo rm -rf /var/lib/apt/lists/*

# . . Install libnabo . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

# (!) ANN was not mentionned in doc
# ANN is a library written in C++, which supports data structures and algorithms for both exact and
# approximate nearest neighbor searching in arbitrarily high dimensions.
# https://www.cs.umd.edu/~mount/ANN/
cd "${ROS_DEV_WORKSPACE}"
wget https://www.cs.umd.edu/~mount/ANN/Files/1.1.2/ann_1.1.2.tar.gz
tar xzf ann_1.1.2.tar.gz
cd ann_1.1.2/
make linux-g++
sudo cp lib/libANN.a /usr/local/lib/
sudo cp include/ANN/ANN.h /usr/local/include/
# shellcheck disable=SC2103
cd ..


# (!) FLANN was not mentionned in doc
# Fast Library for Approximate Nearest Neighbors - development
# FLANN is a library for performing fast approximate nearest neighbor searches
# in high dimensional spaces.
# https://github.com/flann-lib/flann
sudo apt-get update \
    && sudo apt-get install --assume-yes \
        libflann-dev \
    && sudo rm -rf /var/lib/apt/lists/*


cd "${PERCEPT_LIBRARIES}"
# https://github.com/ethz-asl/libnabo
git clone https://github.com/ethz-asl/libnabo.git \
    && cd libnabo \
    && mkdir build && cd build \
    && cmake -D CMAKE_BUILD_TYPE=RelWithDebInfo .. \
    && make -j $(nproc) \
    && make test \
    && sudo make install

#    && git checkout 1.0.7 \

# ToDo:on task end >> next bloc ↓↓
#pwd && tree -L 3

# . . Install percept3D libraries. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
cd "${PERCEPT_LIBRARIES}"

sudo apt-get update \
    && sudo apt-get install --assume-yes \
        libyaml-cpp-dev \
    && sudo rm -rf /var/lib/apt/lists/*

# https://github.com/ethz-asl/libpointmatcher/tree/master
git clone https://github.com/ethz-asl/libpointmatcher.git \
    && cd libpointmatcher \
    && mkdir build && cd build \
    && cmake -D CMAKE_BUILD_TYPE=RelWithDebInfo \
            -D BUILD_TESTS=TRUE \
             .. \
    && make -j $(nproc) \
    && sudo make install

#            -DCMAKE_INSTALL_PREFIX=/usr/local/ \
#    && git checkout 1.3.1 \

cd "${PERCEPT_LIBRARIES}/libpointmatcher/build"
utest/utest --path "${PERCEPT_LIBRARIES}/libpointmatcher/examples/data/"


cd "${PERCEPT_LIBRARIES}"
git clone https://github.com/norlab-ulaval/norlab_icp_mapper.git \
    && mkdir -p norlab_icp_mapper/build && cd norlab_icp_mapper/build \
    && cmake -DCMAKE_BUILD_TYPE=Release .. \
    && make -j $(nproc) \
    && sudo make install


# === ROS =========================================================================================

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

rosdep update
sudo rosdep fix-permissions


# . . Install ROS & build catkin workspace. . . . . . . . . . . . . . . . . . . . . . . . . . . . .
cd "${ROS_DEV_WORKSPACE}"
sudo apt-get update \
    && rosdep install --from-paths ./src --ignore-packages-from-source --rosdistro=${ROS_DISTRO} -y


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
cd "${ROS_DEV_WORKSPACE}/src/"
git clone https://github.com/norlab-ulaval/norlab_icp_mapper_ros.git
git clone https://github.com/norlab-ulaval/libpointmatcher_ros.git
git clone https://github.com/norlab-ulaval/percep3d_turtle_exercises.git


# . . Install ROS & build catkin workspace. . . . . . . . . . . . . . . . . . . . . . . . . . . . .
cd "${ROS_DEV_WORKSPACE}"

# (Priority) ToDo:validate >> next bloc
#export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:${PERCEPT_LIBRARIES}/norlab_icp_mapper

sudo apt-get update
source "${DS_ROS_ROOT}/setup.bash"
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
sudo apt-get update \
    && sudo apt-get install --assume-yes \
        paraview \
    && sudo rm -rf /var/lib/apt/lists/*



#
## https://www.paraview.org
#sudo apt-get update \
#    && sudo apt-get install --assume-yes \
#            python-dev \
#            libgl1-mesa-dev \
#            libxt-dev \
#            qt5-default \
#            libqt5x11extras5-dev \
#            libqt5help5 \
#            qttools5-dev \
#            qtxmlpatterns5-dev-tools \
#            libqt5svg5-dev \
#            libopenmpi-dev \
#            libtbb-dev \
#            ninja-build \
#            xvfb \
#            mesa-utils \
#            python-opengl \
#            ffmpeg \
##          python3-dev \
##          python3-numpy \
#    && sudo rm -rf /var/lib/apt/lists/*
#
#
##wget https://www.paraview.org/paraview-downloads/download.php?submit=Download&version=v5.8&type=binary&os=Linux&downloadFile=ParaView-5.8.1-MPI-Linux-Python2.7-64bit.tar.gz
#
## . . Install ParaView for Linux dependencies: LLVM. . . . . . . . . . . . . . . . . . . . . . . .
## Download the latest LLVM source: https://www.paraview.org/Wiki/ParaView_And_Mesa_3D
#curl -L -O https://github.com/llvm/llvm-project/releases/download/llvmorg-11.0.0/llvm-11.0.0.src.tar.xz \
#    && tar -xvf llvm-11.0.0.src.tar.xz \
#    && mkdir llvm-11.0.0.bld && cd llvm-11.0.0.bld \
#    && cmake -DCMAKE_BUILD_TYPE=Release                     \
#             -DCMAKE_INSTALL_PREFIX=$HOME/llvm/install      \
#             -DLLVM_BUILD_LLVM_DYLIB=ON                     \
#             -DLLVM_ENABLE_RTTI=ON                          \
#             -DLLVM_INSTALL_UTILS=ON                        \
#             -DLLVM_TARGETS_TO_BUILD="ARM;X86;AArch64"      \
#             ../llvm-11.0.0.src                             \
#    && make -j $(nproc) \
#    && sudo make install
#
## . . Installing Mesa llvmpipe and swr drivers . . . . . . . . . . . . . . . . . . . . . . . . . .
#
#
#cd "/opt"
#git clone https://gitlab.kitware.com/paraview/paraview.git \
#    && mkdir paraview_build  \
#    && cd paraview \
#    && git checkout v5.8.0 \
#    && git submodule update --init --recursive \
#    && cd ../paraview_build \
#    && cmake -GNinja -DPARAVIEW_USE_PYTHON=ON -DPARAVIEW_USE_MPI=ON -DVTK_SMP_IMPLEMENTATION_TYPE=TBB -DCMAKE_BUILD_TYPE=Release ../paraview \
#    && ninja


# ==== Final step =================================================================================
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
