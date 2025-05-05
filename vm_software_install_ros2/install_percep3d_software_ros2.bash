#!/bin/bash -i
# =================================================================================================
# Percep3D course software install (ROS2 version)
#
# Maintainer: luc.coupal.1@ulaval.ca
#
# Script usage:
#   1. In the VM, execute the following line in a terminal
#       $ sudo apt-get update && sudo apt-get install --assume-yes git
#       $ cd /opt
#       $ sudo git clone --recurse-submodules https://github.com/norlab-ulaval/percep3d_software.git
#       $ cd percep3d_software/vm_software_install_ros2 && sudo bash install_percep3d_software_ros2.bash
#   2. logout current user and login with user `student` pass `percep3d`
#
# Note on unit test:
#    $ docker pull --platform linux/arm64 ubuntu:20.04
#    $ docker build --platform linux/arm64 -f Dockerfile.test -t percep3d-vm-software-tester-ros2-ubuntu:20.04 .
#    $ docker run -a --name iAmTestROSmelodic4vmContainer -t -i percep3d-vm-software-tester-ros2-ubuntu:20.04
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
apt-get update && \
  apt-get install --assume-yes --no-install-recommends \
      locales \
  && rm -rf /var/lib/apt/lists/* \
  && locale-gen en_US en_US.UTF-8 \
  && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

if [[ "${SHOW_SPLASH_IDU}" == 'true' ]]; then
  n2st::norlab_splash "Percep3D course software install" "https://github.com/norlab-ulaval/percep3d_software"
fi

n2st::print_formated_script_header "install_percep3d_software_ros2.bash"


# ....Auto set ROS distro..........................................................................
n2st::print_formated_script_header "Auto set ROS distro" "."
#n2st::print_msg "Auto set ROS distro"

apt-get update \
  && apt-get install --assume-yes \
      lsb-release \
  && rm -rf /var/lib/apt/lists/*

# Retrieve ubuntu version number: DISTRIB_RELEASE
source /etc/lsb-release
if [[ ${DISTRIB_RELEASE} == '20.04' ]]; then # Ubuntu focal
#  ROS_DISTRO='foxy'
  ROS_DISTRO='galactic'
elif [[ ${DISTRIB_RELEASE} == '22.04' ]]; then # Ubuntu jammy
  ROS_DISTRO='humble'
#  ROS_DISTRO='iron'
elif [[ ${DISTRIB_RELEASE} == '24.04' ]]; then # Ubuntu noble
  ROS_DISTRO='jazzy'
else
  n2st::print_msg_error_and_exit "Ubuntu distro ${DISTRIB_RELEASE} not supported by the installer"
fi
echo "Ubuntu version is ${DISTRIB_RELEASE}, will install ROS2 distro ${ROS_DISTRO}"



# ... Add new user ................................................................................
n2st::print_formated_script_header "Add new user" "."

P3D_USER_HOME="/home/${P3D_USER}"

# $ useradd -s /path/to/shell -d /home/{dirname} -m -G {secondary-group} {username}
useradd -s /bin/bash -d "${P3D_USER_HOME}" -m "${P3D_USER}" \
  && yes "${PASSWORD}" | passwd "${P3D_USER}"
# Add sudo group to P3D_USER
usermod -a -G sudo "${P3D_USER}"
# Note: Add the 'video' groups to new user as it's required for GPU access.
# (not a problem on norlab-og but mandatory on Jetson device)
# Ref: https://forums.developer.nvidia.com/t/how-to-properly-create-new-users/68660/2


# .... Create required dir structure ..............................................................
n2st::print_formated_script_header "Create required dir structure" "."

P3D_ROS_ROOT="/opt/ros/${ROS_DISTRO}"
P3D_ROS_DEV_WORKSPACE="${P3D_USER_HOME}/ros2_ws"

mkdir -p "${P3D_ROS_ROOT}"
mkdir -p "${P3D_ROS_DEV_WORKSPACE}/src"
mkdir -p "${PERCEPT_LIBRARIES_PATH}"
mkdir -p "${P3D_USER_HOME}/percep3d_data"


# . . Add archived files . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
n2st::print_formated_script_header "Add archived files" "."

apt-get update \
    && apt-get install --assume-yes \
        apt-utils \
        zip gzip tar unzip \
    && rm -rf /var/lib/apt/lists/*

cd "${P3DS_PATH}/vm_software_install_ros2" || exit 1

# (CRITICAL) Don't execute `cd` before the folling lines
cp "./beginner_tutorials.zip" "${P3D_ROS_DEV_WORKSPACE}/src"
cp "./percep3d_mapping.zip" "${P3D_ROS_DEV_WORKSPACE}/src"

cd "${P3D_ROS_DEV_WORKSPACE}/src"
unzip beginner_tutorials.zip
unzip percep3d_mapping.zip
rm beginner_tutorials.zip
rm percep3d_mapping.zip




# ==== Install tools ==============================================================================
n2st::print_formated_script_header "Install tools" "."

# skip GUI dialog by setting everything to default
export DEBIAN_FRONTEND=noninteractive

# ... install development utilities ...............................................................
apt-get update \
    && apt-get upgrade --assume-yes \
    && apt-get install --assume-yes \
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
    && rm -rf /var/lib/apt/lists/*



# .... hardware acceleration in VM ................................................................
apt-get update \
    && apt-get install --assume-yes \
        mesa-utils \
    && rm -rf /var/lib/apt/lists/*

( \
  echo "# Turn off hardware acceleration. Workaround for Mesa graphics drivers problem when running from a VM"; \
  echo "# ref: https://wiki.ros.org/rviz/Troubleshooting"; \
  echo "export LIBGL_ALWAYS_SOFTWARE=1"; \
)  >> ${P3D_USER_HOME}/.bashrc



# ===Service: ssh server===========================================================================


n2st::print_formated_script_header "ssh daemon setup" "."

if [[ ${SETUP_SSH_DAEMON} == true ]]; then
  n2st::print_msg "Install and configure ssh daemon"
  VM_SSH_SERVER_PORT=2222

  # install development utilities
  apt-get update \
      && apt-get install --assume-yes  \
          openssh-server \
      && apt-get clean \
      && rm -rf /var/lib/apt/lists/*

  # This will overright the vagrant box VAGRANT_SSH_PORT
  ( \
      echo "LogLevel DEBUG2"; \
      echo "PermitRootLogin yes"; \
      echo "PasswordAuthentication yes"; \
      echo "Port ${VM_SSH_SERVER_PORT}"; \
    ) >> /etc/ssh/sshd_config \
    && mkdir -p /run/sshd

  service ssh --full-restart

  print_msg "Check that ssh is running properly\n\n$(ps -aux | grep -e sshd -e USER)\n"
else
  n2st::print_msg "Skip ssh daemon install and configuration"
  VM_SSH_SERVER_PORT=$VAGRANT_SSH_PORT
fi

# ==== Install percep3D libraries and dependencies ===============================================

# .... Dependencies ...............................................................................
n2st::print_formated_script_header "Install percep3D libraries and dependencies" "."

apt-get update \
    && apt-get install --assume-yes \
        python3-dev \
        python3-opengl \
        python3-numpy \
        python3-pip \
    && rm -rf /var/lib/apt/lists/*;
#        python-is-python3 \

python3 -m pip install --upgrade pip


n2st::print_formated_script_header "Install libpointmatcher" "."

cd "${PERCEPT_LIBRARIES_PATH}"
git clone --recurse-submodules https://github.com/ethz-asl/libpointmatcher.git
cd libpointmatcher

if [[ ${DISTRIB_RELEASE} == '20.04' ]]; then
  # Quick-hack to get the proper get-pip.py version for noetic
  FILE_TO_CHANGE="${PERCEPT_LIBRARIES_PATH}/libpointmatcher/build_system/ubuntu/lpm_install_dependencies_general_ubuntu.bash"
  SEEK_STR="wget https://bootstrap.pypa.io/get-pip.py"
  CHANGE_TO="wget https://bootstrap.pypa.io/pip/3.8/get-pip.py"
  test -f "$FILE_TO_CHANGE" || exit 1
  n2st::seek_and_modify_string_in_file "$SEEK_STR" "$CHANGE_TO" "$FILE_TO_CHANGE"
fi

yes 1 | bash libpointmatcher_dependencies_installer.bash || exit 1

export APPEND_TO_CMAKE_FLAG=( "-D CMAKE_INSTALL_PREFIX=${PERCEPT_LIBRARIES_PATH:?err}" )
# (CRITICAL) ToDo: fix(LPM installer): `--build-system-CI-install` logic should be the default in LPM installer
bash libpointmatcher_installer.bash \
      --compile-test \
      --build-system-CI-install \
      --cmake-build-type Release \
    || exit 1

# (CRITICAL) ToDo: on task end >> unmute next bloc ↓↓
#cd "${PERCEPT_LIBRARIES_PATH}/libpointmatcher/build_system/ubuntu/"
#bash lpm_execute_libpointmatcher_unittest.bash

n2st::print_formated_script_header "Install norlab_icp_mapper" "."
cd "${PERCEPT_LIBRARIES_PATH}"
git clone https://github.com/norlab-ulaval/norlab_icp_mapper.git \
    && mkdir -p norlab_icp_mapper/build && cd norlab_icp_mapper/build \
    && cmake -D CMAKE_BUILD_TYPE=Release \
             -D CMAKE_INSTALL_PREFIX=${PERCEPT_LIBRARIES_PATH:?err} \
             .. \
    && make -j $(nproc) \
    && make install \
    || exit 1


# === ROS =========================================================================================
# //// REFACTORING inprogress ↓↓ /////////////////////////////////////////////////////////////<--//
n2st::print_formated_script_header "Install ROS2" "."

# ... register the ROS package source .............................................................
apt-get update \
    && apt-get upgrade --assume-yes \
    && apt-get install --assume-yes --no-install-recommends \
        software-properties-common \
        python3-argcomplete \
    && rm -rf /var/lib/apt/lists/* \
    && add-apt-repository universe \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null



# ====Install ROS2=================================================================================

# update-alternatives --install /usr/bin/python python /usr/bin/python3 1 \
#   && update-alternatives --install /usr/bin/pip pip /usr/bin/pip3 1


# Note:
#   - ref cmake upgrade via pip:
#        - upgrade cmake - https://stackoverflow.com/a/56690743
#        - this is needed to build some of the ROS2 packages
#        - use pip to upgrade cmake instead because of kitware's rotating GPG keys:
#       Source https://github.com/dusty-nv/jetson-containers/issues/216
#   - pinning 'setuptools' is a quick hack for building ros2 example under ROS foxy focal amd64
#   - pip 'flake8-blind-except' version
#       ref https://github.com/ros2/examples/issues/325#issuecomment-936131710
apt-get update --fix-missing \
  && apt-get install --assume-yes --no-install-recommends \
      apt-utils \
      lsb-release \
      build-essential \
      clang \
      gdb \
      libbullet-dev \
      libpython3-dev \
      python3-vcstools \
      python3-setuptools \
      python3-pycodestyle \
      python3-pygments \
      python3-flake8 \
  && apt-get clean \
  && rm -rf /var/lib/apt/lists/* \
  && python3 -m pip install --upgrade pip \
  && pip3 install --no-cache-dir  \
      scikit-build \
      six \
      msgpack \
  && pip3 install --upgrade --no-cache-dir \
      setuptools==58.2.0 \
      pyparsing==2.4.7 \
  && pip3 install --upgrade --no-cache-dir  \
      cmake

# Note: ROS2 doc recommend executing 'apt upgrade' just before installing ros
apt-get update --fix-missing \
  && apt-get upgrade --assume-yes \
  && apt-get install --assume-yes --no-install-recommends \
      ros-"${ROS_DISTRO}"-"${ROS_PKG}" \
      ros-dev-tools \
  && dpkg --configure -a \
  && apt-get clean \
  && rm -rf /var/lib/apt/lists/*

# Note: pytest version is managed by 'colcon-common-extensions'
apt-get update --fix-missing \
  && apt-get install --assume-yes --no-install-recommends \
      python3-rosdep \
      python3-colcon-common-extensions \
      python3-colcon-mixin \
      libasio-dev \
      libtinyxml2-dev \
      libcunit1-dev \
  && apt-get clean \
  && rm -rf /var/lib/apt/lists/* \
  && echo "Install python testing tools" \
  && pip3 install --upgrade --no-cache-dir \
      flake8-blind-except==0.1.1 \
      flake8-builtins \
      flake8-class-newline \
      flake8-comprehensions \
      flake8-deprecated \
      flake8-docstrings \
      flake8-import-order \
      flake8-quotes \
      importlib-metadata \
      mock \
      nose \
      pep8 \
      pydocstyle \
      pyflakes \
      pytest-mock \
      pytest-repeat \
      pytest-rerunfailures \
      pytest-runner


apt-get update --fix-missing \
  && apt-get install --assume-yes --no-install-recommends \
      ros-"${ROS_DISTRO}"-rosbridge-server \
      ros-"${ROS_DISTRO}"-rqt-graph \
      ros-"${ROS_DISTRO}"-rviz2 \
      tree \
  && apt-get clean \
  && rm -rf /var/lib/apt/lists/*


## ToDo: assessment >> maybe not usefull anymore (pining setuptools as been moved earlier)
# echo "Quick hack for building ros2 example under ROS foxy focal linux/amd64"  \
# && pip3 install setuptools==58.2.0 \
# && apt-get update --fix-missing \
# && apt-get install --assume-yes \
#     python3-catkin-pkg \
#     python3-catkin-pkg-modules \
# && apt-get clean \
# && rm -rf /var/lib/apt/lists/*

# ====Build ROS2 workspace=========================================================================
cd "${P3D_ROS_DEV_WORKSPACE}/src/"

# Note: The goal is to have a minimum ros package to build. Only fetch rclpy as rclcpp is a lot
#        longer to build especily on non-native architecture.
git clone --depth=1 https://github.com/ros2/examples src/ros2_examples -b ${ROS_DISTRO} \
    && cd src/ros2_examples \
    && git sparse-checkout set --no-cone rclpy/topics/minimal_publisher rclpy/topics/minimal_subscriber \
    && cd rclpy/topics/minimal_publisher || exit 1 \
    && cd - && cd rclpy/topics/minimal_subscriber || exit 1


# Note: For some reason, building ros2 example under ROS foxy focal linux/amd64 fail when \
#       "rosdep update" use those flags:
#        $ rosdep update --rosdistro ${ROS_DISTRO} --include-eol-distros \
echo "sourcing /opt/ros/${ROS_DISTRO}/setup.bash" \
    && source /opt/ros/"${ROS_DISTRO}"/setup.bash \
    && apt-get update --fix-missing \
    && rosdep init \
    && rosdep update

    #    && rosdep fix-permissions

colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml  \
    && colcon mixin update  \
    && colcon metadata add default https://raw.githubusercontent.com/colcon/colcon-metadata-repository/master/index.yaml  \
    && colcon metadata update

cd "${P3D_ROS_DEV_WORKSPACE}"

echo "sourcing /opt/ros/${ROS_DISTRO}/setup.bash" \
    && source /opt/ros/"${ROS_DISTRO}"/setup.bash \
    && tree src/ros2_examples \
    && rosdep install  \
            --ignore-packages-from-source \
            --from-path ./src  \
            --rosdistro "${ROS_DISTRO}"  \
            -y \
    && colcon version-check

declare -a COLCON_FLAGS

# Note: The colcon flag "--cmake-clean-cache" reset the python3 env
COLCON_FLAGS=()
COLCON_FLAGS+=(--symlink-install)
#COLCON_FLAGS+=(--executor sequential) # alternative if symlink install is unstable
COLCON_FLAGS+=( \
      --cmake-clean-cache \
      --cmake-args -DCMAKE_BUILD_TYPE=Release \
      --event-handlers console_direct+ \
   )
echo -e "COLCON_FLAGS=(${COLCON_FLAGS[*]})"

echo "sourcing /opt/ros/${ROS_DISTRO}/setup.bash and run colcon build" \
  && source /opt/ros/"${ROS_DISTRO}"/setup.bash \
  && colcon build "${COLCON_FLAGS[@]}"


# ....ROS2 install sanity check....................................................................
echo "sourcing /opt/ros/${ROS_DISTRO}/setup.bash" \
  && source /opt/ros/"${ROS_DISTRO}"/setup.bash \
  && echo "sourcing ${P3D_ROS_DEV_WORKSPACE}/install/setup.bash" \
  && source "${P3D_ROS_DEV_WORKSPACE}"/install/setup.bash \
  && echo "Sanity check" \
  && echo ROS_VERSION="${ROS_VERSION:?'Build argument needs to be set and non-empty.'}" \
  && echo ROS_PYTHON_VERSION="${ROS_PYTHON_VERSION:?'Build argument needs to be set and non-empty.'}" \
  && echo ROS_DISTRO="${ROS_DISTRO:?'Build argument needs to be set and non-empty.'}" \
  && echo PATH="${PATH}" \
  && echo PYTHONPATH="${PYTHONPATH:?'Build argument needs to be set and non-empty.'}" \
  && echo AMENT_PREFIX_PATH="${AMENT_PREFIX_PATH:?'Build argument needs to be set and non-empty.'}" \
  && echo COLCON_PREFIX_PATH="${COLCON_PREFIX_PATH:?'Build argument needs to be set and non-empty.'}" \
  && python -c "import rclpy" \
  && ros2 pkg list \
  && colcon --log-level error test-result --all --verbose \
  || exit 1

echo "Check workspace directory installation"  \
    && [[ -d ${P3D_ROS_DEV_WORKSPACE}/build ]] \
    && [[ -d ${P3D_ROS_DEV_WORKSPACE}/install ]] \
    && [[ -d ${P3D_ROS_DEV_WORKSPACE}/log ]] \
    && [[ -d ${P3D_ROS_DEV_WORKSPACE}/src ]] \
    || exit 1


n2st::print_formated_script_header "Install ROS2: build workspace" "."

echo "source ${P3D_ROS_ROOT}/setup.bash" >> ~/.bashrc
echo "source ${P3D_ROS_DEV_WORKSPACE}/install/setup.bash" >> ~/.bashrc
echo "source ${P3D_ROS_ROOT}/setup.bash" >> "${P3D_USER_HOME}/.bashrc"
echo "source ${P3D_ROS_DEV_WORKSPACE}/install/setup.bash" >> "${P3D_USER_HOME}/.bashrc"


## . . Pull required repository. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
n2st::print_formated_script_header "Pull ROS required repository" "."

cd "${P3D_ROS_DEV_WORKSPACE}/src/"
git clone --branch "${ROS_DISTRO}" https://github.com/norlab-ulaval/norlab_icp_mapper_ros.git
git clone --branch "${ROS_DISTRO}" https://github.com/norlab-ulaval/libpointmatcher_ros.git
git clone https://github.com/norlab-ulaval/percep3d_turtle_exercises.git


# . . Install ROS & build catkin workspace. . . . . . . . . . . . . . . . . . . . . . . . . . . . .
n2st::print_formated_script_header "Install ROS required repository" "."
cd "${P3D_ROS_DEV_WORKSPACE}"

# (Priority) ToDo:validate >> next bloc
#export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:${PERCEPT_LIBRARIES_PATH}/norlab_icp_mapper

# (Priority) ToDo:validate >> next bloc
#export CMAKE_PREFIX_PATH="${PERCEPT_LIBRARIES_PATH:?err}:${CMAKE_PREFIX_PATH}"
#echo "export CMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH}" >> ~/.bashrc
#echo "export CMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH}" >> "${P3D_USER_HOME}/.bashrc"

# Required dependencies for tutorial: Introduction to tf https://wiki.ros.org/tf/Tutorials/Introduction%20to%20tf
# Note: tf is deprecated in favor of tf2 ››› Install tf2 for tutorial in exo module 2.4
apt-get update \
    && apt-get install --assume-yes \
          ros-"${ROS_DISTRO}"-turtle-tf2 \
          ros-"${ROS_DISTRO}"-tf2-tools \
          ros-"${ROS_DISTRO}"-tf \
    && rm -rf /var/lib/apt/lists/*


apt-get update --fix-missing
    rosdep update --rosdistro "${ROS_DISTRO}" --include-eol-distros
    rosdep fix-permissions
    rosdep install \
        --ignore-packages-from-source \
        --from-path ./src  \
        --rosdistro "${ROS_DISTRO}"  \
        -y \
    && colcon version-check

# Note: The colcon flag "--cmake-clean-cache" reset the python3 env
COLCON_FLAGS=()
COLCON_FLAGS+=(--symlink-install)
#COLCON_FLAGS+=(--executor sequential) # alternative if symlink install is unstable
COLCON_FLAGS+=(
      "--cmake-clean-cache"
      "--cmake-args" "-DCMAKE_BUILD_TYPE=Release"
#      "--cmake-args" "-DCMAKE_PREFIX_PATH=${PERCEPT_LIBRARIES_PATH:?err}:${CMAKE_PREFIX_PATH}"
      "--event-handlers" "console_direct+"
   )
echo -e "COLCON_FLAGS=( ${COLCON_FLAGS[*]} )"
colcon build "${COLCON_FLAGS[@]}"

# ///////////////////////////////////////////////////////////////// REFACTORING inprogress ///<--//

# .... install Paraview ...........................................................................
n2st::print_formated_script_header "Install Paraview" "."

apt-get update \
    && apt-get install --assume-yes \
        paraview \
    && rm -rf /var/lib/apt/lists/*


# ....Fetch ros bag husky_short_demo.bag...........................................................
n2st::print_formated_script_header "Fetch ros bag husky_short_demo.bag" "."

cd "${P3D_USER_HOME}/percep3d_data"
wget -O husky_short_demo.zip "http://norlab.s3.valeria.science/percep3d/husky_short_demo.zip?AWSAccessKeyId=XMBLP3A0338XN5LASKV2&Expires=2319980812&Signature=n5HiUTunG7tcTINJovxH%2FtnGbM4%3D"
unzip husky_short_demo.zip
rm husky_short_demo.zip


# ==== Final step =================================================================================
n2st::print_formated_script_header "Finalize percep3d-software-install" "."

# Make sure that you have your environment properly setup. A good way to check is to ensure that
# environment variables like ROS_ROOT and ROS_PACKAGE_PATH are set:
#   $ printenv | grep ROS
printenv | grep ROS

cd "${P3D_ROS_DEV_WORKSPACE}"

# Change directory ownership
chown -R student:student "${P3D_USER_HOME}/percep3d_data"
chown -R student:student "${P3D_ROS_DEV_WORKSPACE}"

n2st::print_msg_done "To connect remotely to the container:
    $ ssh -p ${VM_SSH_SERVER_PORT} ${P3D_USER}@$(hostname -I | awk '{print $1}')
    $ sftp -P ${VM_SSH_SERVER_PORT} openssh-$(hostname -I | awk '{print $1}')
    $ scp -P ${VM_SSH_SERVER_PORT} /path/to/foo ${P3D_USER}@$(hostname -I | awk '{print $1}'):/dest/
"

n2st::print_formated_script_footer "install_percep3d_software_ros2.bash"
# ====Teardown=====================================================================================
cd "${TMP_CWD}" || exit

