#!/bin/bash -i
DOCUMENTATION_INSTALL_PERCEP_3_D_SOFTWARE_ROS_1=$( cat <<'EOF'
# =================================================================================================
# Percep3D course software install (ROS1 version)
#
# Usage:
#   $ bash install_percep3d_software_ros1.bash [--help] [--install-ssh-daemon] [--no-splash]
#
# Arguments:
#   --install-ssh-daemon    Configure and start an ssh daemon on the vm for remote developement
#   --no-splash             Skip the script splash screen (for developer)
#   -h | --help             Script usage with install instruction step reminder
#
# Note on VM script install steps:
#   1. Spin a fresh VM using your prefered virtual machine provider
#   2. Execute the install script with sudo
#   3. Wait for the install script execution end.
#      You will see console message: Completed install_percep3d_software_ros*.bash
#   4. Logout the current user and login with the new user student (password percep3d)
#   5. (optional) If you're using a server version .iso, just run the following line to install a GUI
#       >>> sudo apt-get install --assume-yes --no-install-recommends ubuntu-desktop
#       >>> sudo shutdown --reboot now
#
# =================================================================================================
EOF
)
set -e # Note: we want the installer to always fail-fast (it wont affect the build system policy)


# ....Hardcoded environment variable...............................................................
ROS_PKG='desktop-full'
P3D_USER='student'
PASSWORD='percep3d'
PERCEPT_LIBRARIES_PATH="/opt/percep3d_libraries"
VAGRANT_SSH_PORT=22


# ....Project root logic...........................................................................
TMP_CWD=$(pwd)
P3DS_PATH=$(git rev-parse --show-toplevel)
cd "${P3DS_PATH}" || exit 1

# ....Helper function..............................................................................
N2ST_PATH=${N2ST_PATH:-"${P3DS_PATH}/utilities/norlab-shell-script-tools"}

cd "${N2ST_PATH}" || exit 1
source import_norlab_shell_script_tools_lib.bash


function show_help() {
  # (NICE TO HAVE) ToDo: refactor as a n2st fct (ref NMO-583)
  echo -e "${MSG_DIMMED_FORMAT}"
  n2st::draw_horizontal_line_across_the_terminal_window "="
  echo -e "$0 --help\n"
  # Strip shell comment char `#` and both lines
  echo -e "${DOCUMENTATION_INSTALL_PERCEP_3_D_SOFTWARE_ROS_1}" | sed 's/\# ====.*//' | sed 's/^\#//'
  n2st::draw_horizontal_line_across_the_terminal_window "="
  echo -e "${MSG_END_FORMAT}"
}

# ....Set env variables (pre cli)..................................................................
declare -a REMAINING_ARGS
INSTALL_SSH_DAEMON=false # Skip ssh daemon setup if set to false
SHOW_SPLASH=true


# ....cli..........................................................................................
while [ $# -gt 0 ]; do

  case $1 in
    --install-ssh-daemon)
      INSTALL_SSH_DAEMON=true
      shift # Remove argument (--install-ssh-daemon)
      ;;
    --no-splash)
      SHOW_SPLASH=false
      shift # Remove argument (--no-splash)
      ;;
    -h | --help)
      clear
      show_help
      exit
      ;;
    --) # no more option
      shift
      REMAINING_ARGS=( "$@" )
      break
      ;;
    *) # Default case
      REMAINING_ARGS=("$@")
      break
      ;;
  esac

done

# ....Set env variables (post cli)...............................................................
P3D_USER_HOME="/home/${P3D_USER}"
P3D_ROS_DEV_WORKSPACE="${P3D_USER_HOME}/catkin_ws"
export DEBIAN_FRONTEND=noninteractive

# ....Setup timezone and localization..............................................................
# change the locale from POSIX to UTF-8
apt-get update && \
  apt-get install --assume-yes --no-install-recommends \
      locales \
      lsb-release \
      tree \
  && rm -rf /var/lib/apt/lists/* \
  && locale-gen en_US en_US.UTF-8 \
  && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 LANGUAGE=en_US:en

# Update the current shell
export LC_ALL=en_US.UTF-8
export LANG=en_US.UTF-8
export LANGUAGE=en_US:en

if [[ "${SHOW_SPLASH}" == 'true' ]]; then
  n2st::norlab_splash "Percep3D course software install" "https://github.com/norlab-ulaval/percep3d_software" 2>/dev/null
fi
n2st::print_formated_script_header "install_percep3d_software_ros1.bash"


# ....Auto set ROS distro..........................................................................
n2st::print_formated_script_header "Auto set ROS distro" "."

# Retrieve ubuntu version number: DISTRIB_RELEASE
source /etc/lsb-release
if [[ ${DISTRIB_RELEASE} == '18.04' ]]; then
  P3D_ROS_DISTRO='melodic'
elif [[ ${DISTRIB_RELEASE} == '20.04' ]]; then
  P3D_ROS_DISTRO='noetic'
else
  n2st::print_msg_error_and_exit "Ubuntu distro ${DISTRIB_RELEASE} not supported by the installer"
fi
P3D_ROS_ROOT="/opt/ros/${P3D_ROS_DISTRO}"
n2st::print_msg "Ubuntu version is ${DISTRIB_RELEASE}, will install ROS1 distro ${P3D_ROS_DISTRO} at ${P3D_ROS_ROOT}"


# ====Begin========================================================================================

# ... Add new user ................................................................................
n2st::print_formated_script_header "Add new user" "."

useradd -s /bin/bash -d "${P3D_USER_HOME}" -m "${P3D_USER}" \
  && yes "${PASSWORD}" | passwd "${P3D_USER}"
# Add sudo group to P3D_USER
usermod -a -G sudo "${P3D_USER}"
# Note: Add the 'video' groups to new user as it's required for GPU access.
# (not a problem on norlab-og but mandatory on Jetson device)
# Ref: https://forums.developer.nvidia.com/t/how-to-properly-create-new-users/68660/2


# .... Create required dir structure ..............................................................
n2st::print_formated_script_header "Create required dir structure" "."


mkdir -p "${P3D_ROS_ROOT}"
mkdir -p "${P3D_ROS_DEV_WORKSPACE}/src"
mkdir -p "${PERCEPT_LIBRARIES_PATH}"
mkdir -p "${P3D_USER_HOME}/percep3d_data"

tree -L 1 ${P3D_ROS_ROOT}
tree -L 1 ${P3D_ROS_DEV_WORKSPACE}
tree -L 1 ${PERCEPT_LIBRARIES_PATH}
tree -L 1 ${P3D_USER_HOME}

# . . Add archived files . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
n2st::print_formated_script_header "Add archived files" "."

apt-get update \
    && apt-get install --assume-yes \
        apt-utils \
        zip gzip tar unzip \
    && rm -rf /var/lib/apt/lists/*

cd "${P3DS_PATH}/src/vm_software_install_ros1" || exit 1
cp "./beginner_tutorials.zip" "${P3D_ROS_DEV_WORKSPACE}/src"
cp "./percep3d_mapping.zip" "${P3D_ROS_DEV_WORKSPACE}/src"

cd "${P3D_ROS_DEV_WORKSPACE}/src"
unzip beginner_tutorials.zip
unzip percep3d_mapping.zip
rm beginner_tutorials.zip
rm percep3d_mapping.zip


# ==== Install tools ==============================================================================
n2st::print_formated_script_header "Install tools" "."

# ... install development utilities ...............................................................
apt-get update \
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

if [[ ${INSTALL_SSH_DAEMON} == true ]]; then
  n2st::print_msg "Install and configure ssh daemon"
  VM_SSH_SERVER_PORT=2222

  # install development utilities
  apt-get update \
      && apt-get install --assume-yes  \
          openssh-server \
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

if [[ ${P3D_ROS_DISTRO} == 'melodic' ]]; then
    apt-get update \
        && apt-get install --assume-yes \
            python-dev \
            python-opengl \
            python-numpy \
        && rm -rf /var/lib/apt/lists/*;

    # Required dependencies for tutorial: Introduction to tf https://wiki.ros.org/tf/Tutorials/Introduction%20to%20tf
    # Work around to install pip in python2
    curl https://bootstrap.pypa.io/pip/2.7/get-pip.py --output get-pip.py
    python2 get-pip.py
    python2 -m  pip install --no-cache-dir --verbose \
        pyyaml
else
    apt-get update \
        && apt-get install --assume-yes \
            python3-dev \
            python3-opengl \
            python3-numpy \
            python3-pip \
            python-is-python3 \
        && rm -rf /var/lib/apt/lists/*\
        && python3 -m pip install --upgrade pip
fi

# . . Install boost. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
# https://www.boost.org/doc/libs/1_79_0/more/getting_started/unix-variants.html
apt-get update \
    && apt-get install --assume-yes \
        libboost-all-dev \
    && rm -rf /var/lib/apt/lists/*

# . . Install eigen . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
apt-get update \
    && apt-get install --assume-yes \
        libeigen3-dev \
    && rm -rf /var/lib/apt/lists/*

# . . Install libnabo dependencies  . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

# (!) ANN was not mentionned in doc
# ANN is a library written in C++, which supports data structures and algorithms for both exact and
# approximate nearest neighbor searching in arbitrarily high dimensions.
# https://www.cs.umd.edu/~mount/ANN/
cd "${P3D_ROS_DEV_WORKSPACE}"
wget https://www.cs.umd.edu/~mount/ANN/Files/1.1.2/ann_1.1.2.tar.gz
tar xzf ann_1.1.2.tar.gz
cd ann_1.1.2/
make linux-g++
cp lib/libANN.a /usr/local/lib/
cp include/ANN/ANN.h /usr/local/include/
# shellcheck disable=SC2103
cd ..


# (!) FLANN was not mentionned in doc
# Fast Library for Approximate Nearest Neighbors - development
# FLANN is a library for performing fast approximate nearest neighbor searches
# in high dimensional spaces.
# https://github.com/flann-lib/flann
apt-get update \
    && apt-get install --assume-yes \
        libflann-dev \
    && rm -rf /var/lib/apt/lists/*


# . . Install libnabo . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
cd "${PERCEPT_LIBRARIES_PATH}"
git clone https://github.com/norlab-ulaval/libnabo.git \
    && cd libnabo \
    && git checkout c925c4709a383b702d547993df8842a42bbeb230 \
    && mkdir build && cd build \
    && cmake -D CMAKE_BUILD_TYPE=RelWithDebInfo \
             -D CMAKE_INSTALL_PREFIX=${PERCEPT_LIBRARIES_PATH:?err} \
             .. \
    && make -j $(nproc) \
    && make install \
    || exit 1

#    && make test \

# Note libnabo tags:
#  - Checkout c925c4709a383b702d547993df8842a42bbeb230 -> summer 2022
#  - Checkout 1.0.7 -> Feb 11 2019
#  - Checkout 1.1.0 -> Jan 31 2024

# . . Install percep3D libraries. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
n2st::print_formated_script_header "Install libpointmatcher" "."

apt-get update \
    && apt-get install --assume-yes \
        libyaml-cpp-dev \
    && rm -rf /var/lib/apt/lists/*

cd "${PERCEPT_LIBRARIES_PATH}"
git clone --recurse-submodules https://github.com/norlab-ulaval/libpointmatcher.git \
    && cd libpointmatcher \
    && mkdir build && cd build \
    && git checkout 1.4.0 \
    && cmake -D CMAKE_BUILD_TYPE=RelWithDebInfo \
             -D CMAKE_INSTALL_PREFIX=${PERCEPT_LIBRARIES_PATH:?err} \
             -D BUILD_TESTS=TRUE \
             .. \
    && make -j $(nproc) \
    && make install \
    || exit 1

# Note libpointmatcher tags:
#  - Checkout 1.3.1 -> Mar 4, 2019
#  - Checkout 1.4.0 -> Dec 29, 2023

cd "${PERCEPT_LIBRARIES_PATH}/libpointmatcher/build" || exit 1
utest/utest --path "${PERCEPT_LIBRARIES_PATH}/libpointmatcher/examples/data/"

n2st::print_formated_script_header "Install norlab_icp_mapper" "."
cd "${PERCEPT_LIBRARIES_PATH}"
git clone https://github.com/norlab-ulaval/norlab_icp_mapper.git \
    && cd norlab_icp_mapper \
    && git checkout 7d66288e01f2e4f820edbeb97c5ffe420cc5b230 \
    && mkdir -p build && cd build \
    && cmake -D CMAKE_BUILD_TYPE=Release \
             -D CMAKE_INSTALL_PREFIX=${PERCEPT_LIBRARIES_PATH:?err} \
             .. \
    && make -j $(nproc) \
    && make install \
    || exit 1

# Note norlab_icp_mapper tags:
#  - Checkout 67fc0f892e208d1d5ec973062944d6e88b663baf -> June 3, 2022
#  - Checkout 7d66288e01f2e4f820edbeb97c5ffe420cc5b230 -> Mar 22, 2022


# === ROS =========================================================================================
n2st::print_formated_script_header "Install ROS1" "."

# ... register the ROS package source .............................................................
# Setup sources.lst
sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

## Quick hack for installing ROS melodic on Ubuntu 20.04 Focal
#sh -c 'echo "deb http://packages.ros.org/ros/ubuntu bionic main" > /etc/apt/sources.list.d/ros-latest.list'

# Setup keys
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

# ... Install ROS,  ...............................................................................
mkdir -p "${P3D_ROS_DEV_WORKSPACE}/src/"
cd "${P3D_ROS_DEV_WORKSPACE}/src/"

if [[ ${P3D_ROS_DISTRO} == 'melodic' ]]; then
    apt-get update \
        && apt-get install --assume-yes \
            ros-"${P3D_ROS_DISTRO}"-$(echo "${ROS_PKG}" | tr '_' '-') \
            python-rosdep \
            python-rosinstall-generator \
            python-vcstool \
            python-wstool \
            python-rosinstall \
        && sudo rosdep init \
        || exit 1
else
    apt-get update \
        && apt-get install --assume-yes \
            ros-"${P3D_ROS_DISTRO}"-$(echo "${ROS_PKG}" | tr '_' '-') \
            python3-rosdep \
            python3-rosinstall-generator \
            python3-vcstool \
            python3-wstool \
            python3-rosinstall \
        && sudo rosdep init \
        || exit 1
fi

n2st::print_formated_script_header "Install ROS1: rosdep update & install" "."

apt-get update --fix-missing
apt-get install --assume-yes \
     ros-"${P3D_ROS_DISTRO}"-common-msgs

rosdep update --rosdistro="${P3D_ROS_DISTRO}"
rosdep fix-permissions

cd "${P3D_ROS_DEV_WORKSPACE}"
apt-get update \
    && rosdep install \
      --from-paths ./src \
      --ignore-packages-from-source \
      --rosdistro="${P3D_ROS_DISTRO}" \
      -y \
    || exit 1

n2st::print_formated_script_header "Install ROS1: setup catkin workspace" "."

source "${P3D_ROS_ROOT}/setup.bash"
catkin_make || exit 1
source "${P3D_ROS_DEV_WORKSPACE}/devel/setup.bash"

echo "source ${P3D_ROS_ROOT}/setup.bash" >> "${HOME}/.bashrc"
echo "source ${P3D_ROS_DEV_WORKSPACE}/devel/setup.bash" >> "${HOME}/.bashrc"
echo "source ${P3D_ROS_ROOT}/setup.bash" >> "${P3D_USER_HOME}/.bashrc"
echo "source ${P3D_ROS_DEV_WORKSPACE}/devel/setup.bash" >> "${P3D_USER_HOME}/.bashrc"

# ....ROS1 install sanity check....................................................................
n2st::print_formated_script_header "ROS1 install sanity check" "."

echo "sourcing ${P3D_ROS_ROOT}/setup.bash" \
  && source "${P3D_ROS_ROOT}"/setup.bash \
  && echo "sourcing ${P3D_ROS_DEV_WORKSPACE}/devel/setup.bash" \
  && source "${P3D_ROS_DEV_WORKSPACE}"/devel/setup.bash \
  && echo "" \
  && echo ROS_VERSION="${ROS_VERSION:?'Build argument needs to be set and non-empty.'}" \
  && echo ROS_PYTHON_VERSION="${ROS_PYTHON_VERSION:?'Build argument needs to be set and non-empty.'}" \
  && echo ROS_DISTRO="${ROS_DISTRO:?'Build argument needs to be set and non-empty.'}" \
  && echo PATH="${PATH}" \
  && echo PYTHONPATH="${PYTHONPATH:?'Build argument needs to be set and non-empty.'}" \
  && echo CMAKE_PREFIX_PATH="${CMAKE_PREFIX_PATH:?'Build argument needs to be set and non-empty.'}" \
  && echo "" \
  && python3 -c "import rospy" \
  && [[ "$(rosversion --distro)" == "${P3D_ROS_DISTRO}" ]] \
  || exit 1


echo "Check workspace directory installation"  \
    && [[ -d ${P3D_ROS_DEV_WORKSPACE}/src ]] \
    && [[ -d ${P3D_ROS_DEV_WORKSPACE}/devel ]] \
    || exit 1

# . . Pull percep3d ROS1 required repository. . . . . . . . . . . . . . . . . . . . . . . . . . . .
n2st::print_formated_script_header "Pull percep3d ROS1 required repository" "."

cd "${P3D_ROS_DEV_WORKSPACE}/src/"
git clone https://github.com/norlab-ulaval/libpointmatcher_ros.git
cd libpointmatcher_ros
git checkout 0a9e60e694adb9868a12c31808071eba8f0f2ec9
# Checkout 0a9e60e694adb9868a12c31808071eba8f0f2ec9 -> humble branch (default) Apr 19 2022

cd "${P3D_ROS_DEV_WORKSPACE}/src/"
git clone https://github.com/norlab-ulaval/norlab_icp_mapper_ros.git
cd norlab_icp_mapper_ros
git checkout 8aca7103af013bf13ddd0749e7f4512f9f9c6013
# Checkout 8aca7103af013bf13ddd0749e7f4512f9f9c6013 -> ros2 branch (default) Mar 23, 2022

cd "${P3D_ROS_DEV_WORKSPACE}/src/"
git clone https://github.com/norlab-ulaval/percep3d_turtle_exercises.git


# . . Buildl percep3d ROS1 . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . ..
n2st::print_formated_script_header "Buildl percep3d ROS1 required repository" "."
cd "${P3D_ROS_DEV_WORKSPACE}"

source "${P3D_ROS_ROOT}/setup.bash"
source "${P3D_ROS_DEV_WORKSPACE}/devel/setup.bash"
export CMAKE_PREFIX_PATH="${PERCEPT_LIBRARIES_PATH:?err}:${CMAKE_PREFIX_PATH}"

apt-get update --fix-missing
catkin_make || exit 1

# . . Install required dependencies for tutorial. . . . . . . . . . . . . . . . . . . . . . . . .
# Required dependencies for tutorial: Introduction to tf https://wiki.ros.org/tf/Tutorials/Introduction%20to%20tf
# Note: tf is deprecated in favor of tf2 ››› Install tf2 for tutorial in exo module 2.4
apt-get update \
    && apt-get install --assume-yes \
          ros-"${P3D_ROS_DISTRO}"-turtle-tf2 \
          ros-"${P3D_ROS_DISTRO}"-tf2-tools \
          ros-"${P3D_ROS_DISTRO}"-tf \
    && rm -rf /var/lib/apt/lists/*


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

# Make sure that you have your environment properly setup.
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

n2st::print_formated_script_footer "install_percep3d_software_ros1.bash"
# ====Teardown=====================================================================================
cd "${TMP_CWD}" || exit

