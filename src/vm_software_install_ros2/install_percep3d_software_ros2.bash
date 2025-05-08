#!/bin/bash -i
DOCUMENTATION_INSTALL_PERCEP_3_D_SOFTWARE_ROS_1=$( cat <<'EOF'
# =================================================================================================
# Percep3D course software install (ROS2 version)
#
# Usage:
#   $ bash install_percep3d_software_ros2.bash [--help] [--install-ssh-daemon] [--no-splash]
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
n2st::print_formated_script_header "install_percep3d_software_ros2.bash"


# ....Auto set ROS distro..........................................................................
n2st::print_formated_script_header "Auto set ROS distro" "."

# Retrieve ubuntu version number: DISTRIB_RELEASE
source /etc/lsb-release
if [[ ${DISTRIB_RELEASE} == '20.04' ]]; then # Ubuntu focal
  P3D_ROS_DISTRO='foxy'
  # P3D_ROS_DISTRO='galactic'
elif [[ ${DISTRIB_RELEASE} == '22.04' ]]; then # Ubuntu jammy
  P3D_ROS_DISTRO='humble'
#   P3D_ROS_DISTRO='iron'
elif [[ ${DISTRIB_RELEASE} == '24.04' ]]; then # Ubuntu noble
  P3D_ROS_DISTRO='jazzy'Ulysse962

else
  n2st::print_msg_error_and_exit "Ubuntu distro ${DISTRIB_RELEASE} not supported by the installer"
fi
P3D_ROS_ROOT="/opt/ros/${P3D_ROS_DISTRO}"
n2st::print_msg "Ubuntu version is ${DISTRIB_RELEASE}, will install ROS2 distro ${P3D_ROS_DISTRO} at ${P3D_ROS_ROOT}"


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

# //// TEMP MUTE //////////////////////////////////////////////////////////////////////////////////
# (CRITICAL) ToDo: skip installing those 2 zip files until they are refactored for ROS2
#cd "${P3DS_PATH}/src/vm_software_install_ros2" || exit 1
#cp "./beginner_tutorials.zip" "${P3D_ROS_DEV_WORKSPACE}/src"
#cp "./percep3d_mapping.zip" "${P3D_ROS_DEV_WORKSPACE}/src"
#
#cd "${P3D_ROS_DEV_WORKSPACE}/src"
#unzip beginner_tutorials.zip
#unzip percep3d_mapping.zip
#rm beginner_tutorials.zip
#rm percep3d_mapping.zip
# \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\ TEMP MUTE \\\\


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

apt-get update \
  && apt-get upgrade --assume-yes \
    && apt-get install --assume-yes \
        python3-dev \
        python3-opengl \
        python3-numpy \
        python3-pip \
        python-is-python3 \
    && rm -rf /var/lib/apt/lists/* \
    && python3 -m pip install --upgrade pip

n2st::print_formated_script_header "Install libpointmatcher" "."

cd "${PERCEPT_LIBRARIES_PATH}"
git clone --recurse-submodules https://github.com/norlab-ulaval/libpointmatcher.git
cd libpointmatcher

if [[ ${DISTRIB_RELEASE} == '20.04' ]]; then
  # Quick-hack to get the proper get-pip.py version for noetic
  FILE_TO_CHANGE="${PERCEPT_LIBRARIES_PATH}/libpointmatcher/build_system/ubuntu/lpm_install_dependencies_general_ubuntu.bash"
  SEEK_STR="wget https://bootstrap.pypa.io/get-pip.py"
  CHANGE_TO="wget https://bootstrap.pypa.io/pip/3.8/get-pip.py"
  test -f "$FILE_TO_CHANGE" || exit 1
  n2st::seek_and_modify_string_in_file "$SEEK_STR" "$CHANGE_TO" "$FILE_TO_CHANGE"
fi


# //// Temp solution //////////////////////////////////////////////////////////////////////////////
## (Priority) ToDo: unmute when installer as the option to skip doc install (ref task NMO-605).
##
#yes | bash libpointmatcher_dependencies_installer.bash || exit 1
##
## Use manual install step in the mean time
cd "${PERCEPT_LIBRARIES_PATH}"/libpointmatcher/build_system/ubuntu || exit 1
yes | source lpm_install_dependencies_general_ubuntu.bash || exit 1
cd "${PERCEPT_LIBRARIES_PATH}"/libpointmatcher/build_system/ubuntu || exit 1
yes | source lpm_install_dependencies_libnabo_ubuntu.bash || exit 1
# \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\ Temp solution \\\\


cd "${PERCEPT_LIBRARIES_PATH}/libpointmatcher"
export APPEND_TO_CMAKE_FLAG=( "-D CMAKE_INSTALL_PREFIX=${PERCEPT_LIBRARIES_PATH:?err}" )
# (CRITICAL) ToDo: fix(LPM installer): `--build-system-CI-install` logic should be the default
#   in LPM installer.
#   Ref task NMO-602 fix(installer): set LPM user installer to skip repo clone step by default
bash libpointmatcher_installer.bash \
      --compile-test \
      --build-system-CI-install \
      --cmake-build-type Release \
    || exit 1


cd "${PERCEPT_LIBRARIES_PATH}/libpointmatcher/build_system/ubuntu/"
bash lpm_execute_libpointmatcher_unittest.bash

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
n2st::print_formated_script_header "Install ROS2" "."

# ... register the ROS package source .............................................................
apt-get update \
    && apt-get upgrade --assume-yes \
    && apt-get install --assume-yes \
        software-properties-common \
        python3-argcomplete \
    && rm -rf /var/lib/apt/lists/* \
    && add-apt-repository -y universe \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo ${UBUNTU_CODENAME}) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null


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
  && apt-get install --assume-yes \
      clang \
      gdb \
      libbullet-dev \
      libpython3-dev \
      python3-vcstools \
      python3-setuptools \
      python3-pycodestyle \
      python3-pygments \
      python3-flake8 \
  && rm -rf /var/lib/apt/lists/* \
  && python3 -m pip install --upgrade pip \
  && pip3 install --no-cache-dir  \
      scikit-build \
      six \
      msgpack \
  && pip3 install --upgrade --no-cache-dir  \
      cmake

#  && pip3 install --upgrade --no-cache-dir \
#      setuptools==58.2.0 \
#      pyparsing==2.4.7 \

# Note: ROS2 doc recommend executing 'apt upgrade' just before installing ros
apt-get update --fix-missing \
  && apt-get upgrade --assume-yes \
  && apt-get install --assume-yes \
      ros-"${P3D_ROS_DISTRO}"-$(echo "${ROS_PKG}" | tr '_' '-') \
      ros-dev-tools \
  && dpkg --configure -a \
  && rm -rf /var/lib/apt/lists/*

# Note: pytest version is managed by 'colcon-common-extensions'
apt-get update --fix-missing \
  && apt-get install --assume-yes \
      python3-rosdep \
      python3-colcon-common-extensions \
      python3-colcon-mixin \
      libasio-dev \
      libtinyxml2-dev \
      libcunit1-dev \
  && rm -rf /var/lib/apt/lists/* \
  && echo "Install python testing tools" \
  && pip3 install --upgrade --no-cache-dir \
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

#      flake8-blind-except==0.1.1 \

apt-get update --fix-missing \
  && apt-get install --assume-yes --no-install-recommends \
      ros-"${P3D_ROS_DISTRO}"-rosbridge-server \
      ros-"${P3D_ROS_DISTRO}"-rqt-graph \
      ros-"${P3D_ROS_DISTRO}"-rviz2 \
  && rm -rf /var/lib/apt/lists/*


## ToDo: assessment >> maybe not usefull anymore (pining setuptools as been moved earlier)
# echo "Quick hack for building ros2 example under ROS foxy focal linux/amd64"  \
# && pip3 install setuptools==58.2.0 \
# && apt-get update --fix-missing \
# && apt-get install --assume-yes \
#     python3-catkin-pkg \
#     python3-catkin-pkg-modules \
# && rm -rf /var/lib/apt/lists/*



# ====Build ROS2 workspace=========================================================================
n2st::print_formated_script_header "Build ROS2 workspace" "."
cd "${P3D_ROS_DEV_WORKSPACE}/src/"

# //// Consider fetching the full repo ////////////////////////////////////////////////////////////
# Note: The goal is to have a minimum ros package to build. Only fetch rclpy as rclcpp is a lot
#        longer to build especily on non-native architecture.
mkdir -p "${P3D_ROS_DEV_WORKSPACE}/src/ros2_examples"
git clone --depth=1 https://github.com/ros2/examples ros2_examples -b "${P3D_ROS_DISTRO}" \
    && cd ros2_examples \
    && git sparse-checkout set --no-cone rclpy/topics/minimal_publisher rclpy/topics/minimal_subscriber \
    && cd rclpy/topics/minimal_publisher || exit 1 \
    && cd - && cd rclpy/topics/minimal_subscriber || exit 1

## Note: The goal is to have a minimum ros package to build.
#mkdir -p "${P3D_ROS_DEV_WORKSPACE}/src/ros2_examples"
#git clone --branch "${P3D_ROS_DISTRO}" https://github.com/ros2/examples ros2_examples
# \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\ Consider fetching the full repo \\\\

# Note: For some reason, building ros2 example under ROS foxy focal linux/amd64 fail when rosdep
#       update use those flags: $ rosdep update --rosdistro ${P3D_ROS_DISTRO} --include-eol-distros
echo && n2st::print_msg "Run rosdep init" \
    && cd "${P3D_ROS_DEV_WORKSPACE}" \
    && source "/opt/ros/${P3D_ROS_DISTRO}/setup.bash" \
    && apt-get update --fix-missing \
    && sudo rosdep init \
    && rosdep update --rosdistro "${P3D_ROS_DISTRO}" \
    && rosdep fix-permissions \
    || exit 1


colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml  \
    && colcon mixin update  \
    && colcon metadata add default https://raw.githubusercontent.com/colcon/colcon-metadata-repository/master/index.yaml  \
    && colcon metadata update  \
    || exit 1

echo && n2st::print_msg "Run rosdep install" \
    && source "/opt/ros/${P3D_ROS_DISTRO}/setup.bash" \
    && cd "${P3D_ROS_DEV_WORKSPACE}" \
    && tree -L 2 \
    && rosdep install  \
            --ignore-packages-from-source \
            --from-path src  \
            --rosdistro "${P3D_ROS_DISTRO}" \
            -y \
    && colcon version-check \
    || exit 1

#            --include-eol-distros \


# Note: The colcon flag "--cmake-clean-cache" reset the python3 env
COLCON_FLAGS=()
COLCON_FLAGS+=(--symlink-install)
#COLCON_FLAGS+=(--executor sequential) # alternative if symlink install is unstable
COLCON_FLAGS+=( \
      "--cmake-clean-cache" \
      "--cmake-args" "-DCMAKE_BUILD_TYPE=Release" \
      "--event-handlers" "console_direct+" \
   )
echo && n2st::print_msg "Run colcon build ${COLCON_FLAGS[*]}" \
    && source "/opt/ros/${P3D_ROS_DISTRO}/setup.bash" \
    && cd "${P3D_ROS_DEV_WORKSPACE}" \
    && colcon build "${COLCON_FLAGS[@]}" \
    || exit 1


# ....ROS2 install sanity check....................................................................
n2st::print_formated_script_header "ROS2 install sanity check" "."

echo "sourcing /opt/ros/${P3D_ROS_DISTRO}/setup.bash" \
  && source "/opt/ros/${P3D_ROS_DISTRO}/setup.bash" \
  && echo "sourcing ${P3D_ROS_DEV_WORKSPACE}/install/setup.bash" \
  && source "${P3D_ROS_DEV_WORKSPACE}"/install/setup.bash \
  && echo "Sanity check" \
  && echo "" \
  && echo ROS_VERSION="${ROS_VERSION:?'Build argument needs to be set and non-empty.'}" \
  && echo ROS_PYTHON_VERSION="${ROS_PYTHON_VERSION:?'Build argument needs to be set and non-empty.'}" \
  && echo ROS_DISTRO="${ROS_DISTRO:?'Build argument needs to be set and non-empty.'}" \
  && echo PATH="${PATH}" \
  && echo PYTHONPATH="${PYTHONPATH:?'Build argument needs to be set and non-empty.'}" \
  && echo AMENT_PREFIX_PATH="${AMENT_PREFIX_PATH:?'Build argument needs to be set and non-empty.'}" \
  && echo COLCON_PREFIX_PATH="${COLCON_PREFIX_PATH:?'Build argument needs to be set and non-empty.'}" \
  && echo "" \
  && python -c "import rclpy" \
  && ros2 pkg list \
  && echo "" \
  && cd "${P3D_ROS_DEV_WORKSPACE}" \
  && colcon --log-level error test-result --all --verbose \
  || exit 1

echo "Check workspace directory installation"  \
    && [[ -d ${P3D_ROS_DEV_WORKSPACE}/build ]] \
    && [[ -d ${P3D_ROS_DEV_WORKSPACE}/install ]] \
    && [[ -d ${P3D_ROS_DEV_WORKSPACE}/log ]] \
    && [[ -d ${P3D_ROS_DEV_WORKSPACE}/src ]] \
    || exit 1

echo "source ${P3D_ROS_ROOT}/setup.bash" >> "${HOME}/.bashrc"
echo "source ${P3D_ROS_DEV_WORKSPACE}/install/setup.bash" >> "${HOME}/.bashrc"
echo "source ${P3D_ROS_ROOT}/setup.bash" >> "${P3D_USER_HOME}/.bashrc"
echo "source ${P3D_ROS_DEV_WORKSPACE}/install/setup.bash" >> "${P3D_USER_HOME}/.bashrc"


# ====Install Percep3D ROS2 app====================================================================
n2st::print_formated_script_header "Install Percep2D ROS2 app" "-"

# . . Install percep3d required ros2 package . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
n2st::print_formated_script_header "Install percep3d required ros2 package" "."

# Required dependencies for tutorial: Introduction to tf https://wiki.ros.org/tf/Tutorials/Introduction%20to%20tf
# Note: tf is deprecated in favor of tf2 ››› Install tf2 for tutorial in exo module 2.4
apt-get update \
    && apt-get install --assume-yes \
          ros-"${P3D_ROS_DISTRO}"-turtle-tf2-py \
          ros-"${P3D_ROS_DISTRO}"-tf2-ros \
          ros-"${P3D_ROS_DISTRO}"-tf2-tools \
          ros-"${P3D_ROS_DISTRO}"-turtlesim \
    && rm -rf /var/lib/apt/lists/*

# . . Pull percep3d ros2 required repository. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
n2st::print_formated_script_header "Pull percep3d ros2 required repository" "."

cd "${P3D_ROS_DEV_WORKSPACE}/src/"
git clone --branch "${P3D_ROS_DISTRO}" https://github.com/norlab-ulaval/norlab_icp_mapper_ros.git
git clone --branch "${P3D_ROS_DISTRO}" https://github.com/norlab-ulaval/libpointmatcher_ros.git

# //// TEMP MUTE //////////////////////////////////////////////////////////////////////////////////
## (CRITICAL) ToDo: skip installing this repo until he is refactored for ROS2echo
#git clone https://github.com/norlab-ulaval/percep3d_turtle_exercises.git
# \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\ TEMP MUTE \\\\


# . . Build percep3d ros2 required packages . . . . . . . . . . . . . . . . . . . . . . . . . . . .
n2st::print_formated_script_header "Build percep3d ros2 required packages" "."
cd "${P3D_ROS_DEV_WORKSPACE}"

source "/opt/ros/${P3D_ROS_DISTRO}/setup.bash"
source "${P3D_ROS_DEV_WORKSPACE}/install/setup.bash"
export CMAKE_PREFIX_PATH="${PERCEPT_LIBRARIES_PATH:?err}:${CMAKE_PREFIX_PATH}"

echo && n2st::print_msg "Run rosdep install" \
    && apt-get update --fix-missing \
    && cd "${P3D_ROS_DEV_WORKSPACE}" \
    && rosdep update --rosdistro "${P3D_ROS_DISTRO}" \
    && rosdep fix-permissions \
    && rosdep install \
        --ignore-packages-from-source \
        --from-path ./src  \
        --rosdistro "${P3D_ROS_DISTRO}"  \
        -y \
    && colcon version-check \
    || exit 1


# Note: The colcon flag "--cmake-clean-cache" reset the python3 env
COLCON_FLAGS=()
COLCON_FLAGS+=(--symlink-install)
#COLCON_FLAGS+=(--executor sequential) # alternative if symlink install is unstable
COLCON_FLAGS+=(
      "--cmake-clean-cache"
      "--cmake-args" "-DCMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH}"
      "--event-handlers" "console_direct+"
   )
echo && n2st::print_msg "Run colcon build ${COLCON_FLAGS[*]}" \
    && cd "${P3D_ROS_DEV_WORKSPACE}" \
    && colcon build "${COLCON_FLAGS[@]}" \
    || exit 1

# .... install Paraview ...........................................................................
n2st::print_formated_script_header "Install Paraview" "."

apt-get update \
    && apt-get install --assume-yes \
        paraview \
    && rm -rf /var/lib/apt/lists/*


# ....Fetch ros bag husky_short_demo.bag...........................................................
n2st::print_formated_script_header "Fetch ros bag husky_short_demo.bag" "."

# (CRITICAL) ToDo: update this link to the version refactored for ROS2
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

n2st::print_formated_script_footer "install_percep3d_software_ros2.bash"
# ====Teardown=====================================================================================
cd "${TMP_CWD}" || exit

