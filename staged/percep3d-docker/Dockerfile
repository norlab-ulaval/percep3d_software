
# /// percep3d_software course //////////////////////////////////////////////////////////////////////////////////

#ARG BASE_IMAGE=osrf/ros:melodic-desktop-full-bionic
#ARG BASE_IMAGE=nvcr.io/nvidia/l4t-base:r32.6.1
#ARG BASE_IMAGE=ubuntu:18.04
ARG BASE_IMAGE=ubuntu:20.04
FROM ${BASE_IMAGE} AS general-dev-util
MAINTAINER luc.coupal.1@ulaval.ca

ARG ROS_PKG=desktop_full
ENV ROS_PKG=${ROS_PKG}
# melodic
ARG ROS_DISTRO=noetic
ENV ROS_DISTRO={ROS_DISTRO}
ENV DS_ROS_ROOT="/opt/ros/${ROS_DISTRO}"


# ...Add new user.......................................................................................................
ARG NEW_USER=student
ENV P3D_USER=${NEW_USER}
ARG PASSWORD=percept3d
ENV P3D_USER_HOME="/home/${P3D_USER}"

RUN useradd -d "${P3D_USER_HOME}" -m "${P3D_USER}" \
  && yes "${PASSWORD}" | passwd "${P3D_USER}"
# Add the 'video' groups to new user as it's required for GPU access.
# (not a problem on norlab-og but mandatory on Jetson device)
# Ref: https://forums.developer.nvidia.com/t/how-to-properly-create-new-users/68660/2
RUN usermod -a -G sudo ${P3D_USER}

# ...root config........................................................................................................
# user:newpassword
RUN echo "root:${PASSWORD}" | chpasswd

# (Optional) Change default shell for new user
#RUN usermod -s /bin/bash ${P3D_USER}

# .... Create required dir structure ...................................................................................
ENV PERCEPT_LIBRARIES="/opt/percep3d_libraries"
ENV ROS_DEV_WORKSPACE="${P3D_USER_HOME}/catkin_ws"

RUN mkdir -p "${P3D_USER_HOME}/percep3d_data"
WORKDIR "${ROS_DEV_WORKSPACE}"

# ==== Install tools ===================================================================================================
# skip GUI dialog by setting everything to default
ARG DEBIAN_FRONTEND=noninteractive

# install development utilities
RUN apt-get update \
    && apt-get install --assume-yes  \
        apt-utils \
        lsb-release \
        checkinstall  \
        pkg-config \
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
        zip gzip tar unzip \
        bash-completion \
    && rm -rf /var/lib/apt/lists/*



# .... Install percept3D libraries dependencies ........................................................................
#RUN apt-get update \
#    && apt-get install --assume-yes \
#        python-numpy \
#        python3-numpy \
#        python-dev \
#        python3-dev \
#        python-opengl \
#        python3-opengl \
#    && rm -rf /var/lib/apt/lists/*


RUN /bin/bash -c "if [[ ${ROS_DISTRO} == 'melodic' ]]; then \
        apt-get update \
            && apt-get install --assume-yes \
                python-dev \
                python-opengl \
                python-numpy \
            && rm -rf /var/lib/apt/lists/* ; \
    else \
        apt-get update \
            && apt-get install --assume-yes \
                python3-dev \
                python3-opengl \
                python3-numpy \
            && rm -rf /var/lib/apt/lists/* ; \
    fi"


FROM general-dev-util AS percept3d-requirement
# . . Install boost. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
# https://www.boost.org/doc/libs/1_79_0/more/getting_started/unix-variants.html
RUN apt-get update \
    && apt-get install --assume-yes  \
        libboost-all-dev \
    && rm -rf /var/lib/apt/lists/*

    #        --no-install-recommends \
#        --fix-missing \


# . . Install eigen . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . ..
RUN apt-get update \
    && apt-get install --assume-yes \
        libeigen3-dev \
    && rm -rf /var/lib/apt/lists/*

# . . Install libnabo . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . ..

# (!) ANN was not mentionned in doc
# ANN is a library written in C++, which supports data structures and algorithms for both exact and approximate nearest neighbor searching in arbitrarily high dimensions.
# https://www.cs.umd.edu/~mount/ANN/
WORKDIR "${ROS_DEV_WORKSPACE}"
RUN wget https://www.cs.umd.edu/~mount/ANN/Files/1.1.2/ann_1.1.2.tar.gz \
    && tar xzf ann_1.1.2.tar.gz \
    && cd ann_1.1.2/ \
    && make linux-g++ \
    && cp lib/libANN.a /usr/local/lib/ \
    && cp include/ANN/ANN.h /usr/local/include/ \
    && cd ..


# (!) FLANN was not mentionned in doc
# Fast Library for Approximate Nearest Neighbors - development
# FLANN is a library for performing fast approximate nearest neighbor searches in high dimensional spaces.
# https://github.com/flann-lib/flann
RUN apt-get update \
    && apt-get install --assume-yes \
        libflann-dev \
    && rm -rf /var/lib/apt/lists/*


WORKDIR "${PERCEPT_LIBRARIES}"
# https://github.com/ethz-asl/libnabo
RUN git clone https://github.com/ethz-asl/libnabo.git \
    && cd libnabo \
    && mkdir build && cd build \
    && cmake -D CMAKE_BUILD_TYPE=Release .. \
    && make -j $(nproc) \
    && make test \
    && make install

#    && git checkout 1.0.7 \

# ToDo:on task end >> next bloc ↓↓
##RUN pwd && tree -L 3

# . . Install percept3D libraries. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
WORKDIR "${PERCEPT_LIBRARIES}"

RUN apt-get update \
    && apt-get install --assume-yes \
        libyaml-cpp-dev \
    && rm -rf /var/lib/apt/lists/*

FROM percept3d-requirement AS percept3d-tools
# https://github.com/ethz-asl/libpointmatcher/tree/master
# (CRITICAL) ToDo:investigate?? >> next bloc ↓↓
#RUN git clone https://github.com/ethz-asl/libpointmatcher.git \
RUN git clone https://github.com/RedLeader962/libpointmatcher.git \
    && cd libpointmatcher \
    && mkdir build && cd build \
    && cmake -D CMAKE_BUILD_TYPE=Release \
            -D BUILD_TESTS=TRUE \
             .. \
    && make -j $(nproc) \
    && make install

#            -DCMAKE_INSTALL_PREFIX=/usr/local/ \
#    && git checkout 1.3.1 \

WORKDIR "${PERCEPT_LIBRARIES}/libpointmatcher/build"
RUN utest/utest --path "${PERCEPT_LIBRARIES}/libpointmatcher/examples/data/"


WORKDIR "${PERCEPT_LIBRARIES}"
RUN git clone https://github.com/norlab-ulaval/norlab_icp_mapper.git \
    && mkdir -p norlab_icp_mapper/build && cd norlab_icp_mapper/build \
    && cmake -DCMAKE_BUILD_TYPE=Release .. \
    && make -j $(nproc) \
    && make install


# ToDo:on task end >> next bloc ↓↓
#RUN pwd && tree -L 3

FROM percept3d-tools AS ROS-install
# === ROS ==============================================================================================================

# ... register the ROS package source ..................................................................................
# Setup sources.lst
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
# Setup keys
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -


# ... Install ROS,  ....................................................................................................

# Credit for the next two RUN step: NVIDIA-AI-IOT/ros2_jetson
#    https://github.com/NVIDIA-AI-IOT/ros2_jetson/blob/main/docker/DockerFile.l4tbase.ros.noetic

WORKDIR "${ROS_DEV_WORKSPACE}/src/"

RUN /bin/bash -c "if [[ ${ROS_DISTRO} == 'melodic' ]]; then \
        apt-get update \
            && apt-get install --assume-yes \
                ros-${ROS_DISTRO}-$(echo "${ROS_PKG}" | tr '_' '-') \
                python-rosdep \
                python-rosinstall-generator \
                python-vcstool \
                python-wstool \
                python-rosinstall \
            && rosdep init \
            && rosdep update \
            && rosdep fix-permissions \
            && rm -rf /var/lib/apt/lists/*; \
    else \
        apt-get update \
            && apt-get install --assume-yes \
                ros-${ROS_DISTRO}-$(echo "${ROS_PKG}" | tr '_' '-') \
                python3-rosdep \
                python3-rosinstall-generator \
                python3-vcstool \
                python3-wstool \
                python3-rosinstall \
            && rosdep init \
            && rosdep update \
            && rosdep fix-permissions \
            && rm -rf /var/lib/apt/lists/*; \
    fi"


# . . Install ROS & build catkin workspace. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
WORKDIR "${ROS_DEV_WORKSPACE}"
RUN apt-get update \
    && rosdep install --from-paths ./src --ignore-packages-from-source --rosdistro=${ROS_DISTRO} -y

#    && cd ${DS_DEV_WORKSPACE} \
#    && apt-get update \
#    && rosdep install --from-path src --ignore-src --default-yes \
#    && rm -rf /var/lib/apt/lists/*


RUN /bin/bash -c "source ${DS_ROS_ROOT}/setup.bash \
    && catkin_make \
    && source ${ROS_DEV_WORKSPACE}/devel/setup.bash" \
    && echo "source ${DS_ROS_ROOT}/setup.bash" >> ~/.bashrc \
    && echo "source ${ROS_DEV_WORKSPACE}/devel/setup.bash" >> ~/.bashrc
# Make sure your workspace is properly overlayed by the setup script by checking the ROS_PACKAGE_PATH environment
# variable. It should include the directory you're in:
#   $ echo $ROS_PACKAGE_PATH
#   > /home/youruser/ros_catkin_ws/src:/opt/ros/melodic/share


FROM ROS-install AS percept3d-ROS
## . . Pull required repository. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
WORKDIR "${ROS_DEV_WORKSPACE}/src/"
RUN git clone https://github.com/norlab-ulaval/norlab_icp_mapper_ros.git
RUN git clone https://github.com/norlab-ulaval/libpointmatcher_ros.git
RUN git clone https://github.com/norlab-ulaval/percep3d_turtle_exercises.git

# . . Add archived files . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
COPY beginner_tutorials.zip "${ROS_DEV_WORKSPACE}/src"
COPY percep3d_mapping.zip "${ROS_DEV_WORKSPACE}/src"
RUN unzip beginner_tutorials.zip
RUN unzip percep3d_mapping.zip


# . . Install ROS & build catkin workspace. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
WORKDIR "${ROS_DEV_WORKSPACE}"

# (Priority) ToDo:validate >> next bloc
#export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:${PERCEPT_LIBRARIES}/norlab_icp_mapper

RUN apt-get update \
    && /bin/bash -c "source ${DS_ROS_ROOT}/setup.bash \
    && catkin_make \
    && source ${ROS_DEV_WORKSPACE}/devel/setup.bash"


## Already install in ROS desktop full install
#RUN apt-get update \
#    && apt-get install -y rviz


FROM percept3d-ROS AS percept3d-ROS-ssh
# ===Service: ssh server================================================================================================

# install development utilities
RUN apt-get update \
    && apt-get install --assume-yes --no-install-recommends \
        openssh-server \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*


# ...Setup ssh server...................................................................................................
# (Priority) ToDo:validate >> SSH bloc ↓↓
# ssh port, remaped from default 22 to 2222
ARG SSH_SERVER_PORT=2222
ENV P3D_SSH_SERVER_PORT=${SSH_SERVER_PORT}
EXPOSE ${SSH_SERVER_PORT}

# Inspired from https://austinmorlan.com/posts/docker_clion_development/
RUN ( \
    echo "LogLevel DEBUG2"; \
    echo "PermitRootLogin yes"; \
    echo "PasswordAuthentication yes"; \
    echo "Port ${SSH_SERVER_PORT}"; \
    echo "Subsystem sftp /usr/lib/openssh/sftp-server"; \
  ) > /etc/ssh/sshd_config_ros4percept3d_openssh_server \
  && mkdir /run/sshd

# SSH login fix. Otherwise user is kicked off after login
RUN sed 's@session\s*required\s*pam_loginuid.so@session optional pam_loginuid.so@g' -i /etc/pam.d/sshd


### .... install Paraview ................................................................................................
# Note | There exist several Docker image for Paraview

##FROM percept3d-ROS AS ROS4percept3DwParaView

### https://www.paraview.org
##RUN apt-get update \
##    && apt-get install --assume-yes \
##            python-dev \
##            libgl1-mesa-dev \
##            libxt-dev \
##            qt5-default \
##            libqt5x11extras5-dev \
##            libqt5help5 \
##            qttools5-dev \
##            qtxmlpatterns5-dev-tools \
##            libqt5svg5-dev \
##            libopenmpi-dev \
##            libtbb-dev \
##            ninja-build \
##            xvfb \
##            mesa-utils \
##            python-opengl \
##            ffmpeg \
###          python3-dev \
###          python3-numpy \
##    && rm -rf /var/lib/apt/lists/*
##
##
###RUN wget https://www.paraview.org/paraview-downloads/download.php?submit=Download&version=v5.8&type=binary&os=Linux&downloadFile=ParaView-5.8.1-MPI-Linux-Python2.7-64bit.tar.gz
##
### . . Install ParaView for Linux dependencies: LLVM. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
### Download the latest LLVM source: https://www.paraview.org/Wiki/ParaView_And_Mesa_3D
##RUN curl -L -O https://github.com/llvm/llvm-project/releases/download/llvmorg-11.0.0/llvm-11.0.0.src.tar.xz \
##    && tar -xvf llvm-11.0.0.src.tar.xz \
##    && mkdir llvm-11.0.0.bld && cd llvm-11.0.0.bld \
##    && cmake -DCMAKE_BUILD_TYPE=Release                     \
##             -DCMAKE_INSTALL_PREFIX=$HOME/llvm/install      \
##             -DLLVM_BUILD_LLVM_DYLIB=ON                     \
##             -DLLVM_ENABLE_RTTI=ON                          \
##             -DLLVM_INSTALL_UTILS=ON                        \
##             -DLLVM_TARGETS_TO_BUILD="ARM;X86;AArch64"      \
##             ../llvm-11.0.0.src                             \
##    && make -j $(nproc) \
##    && make install
##
### . . Installing Mesa llvmpipe and swr drivers . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
##
##
##WORKDIR "/opt"
##RUN git clone https://gitlab.kitware.com/paraview/paraview.git \
##    && mkdir paraview_build  \
##    && cd paraview \
##    && git checkout v5.8.0 \
##    && git submodule update --init --recursive \
##    && cd ../paraview_build \
##    && cmake -GNinja -DPARAVIEW_USE_PYTHON=ON -DPARAVIEW_USE_MPI=ON -DVTK_SMP_IMPLEMENTATION_TYPE=TBB -DCMAKE_BUILD_TYPE=Release ../paraview \
##    && ninja



# ==== Final step ======================================================================================================
# Make sure that you have your environment properly setup. A good way to check is to ensure that environment variables
# like ROS_ROOT and ROS_PACKAGE_PATH are set:
#   $ printenv | grep ROS
WORKDIR "${ROS_DEV_WORKSPACE}"
ENTRYPOINT [ "./entrypoint.bash" ]
CMD [ "bash" ]
# ///////////////////////////////////////////////////////////////////////////////////////// norlab-mppi/dependencies ///

