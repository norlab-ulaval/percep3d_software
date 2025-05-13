<div align="center">

[//]: # ( ==== Logo ================================================== ) 
<br>
<br>
<a href="https://norlab.ulaval.ca">
    <picture>
      <source media="(prefers-color-scheme: dark)" srcset="/visual/norlab_logo_acronym_light.png">
      <source media="(prefers-color-scheme: light)" srcset="/visual/norlab_logo_acronym_dark.png">
      <img alt="Shows an the dark NorLab logo in light mode and light NorLab logo in dark mode." src="/visual/norlab_logo_acronym_dark.png" width="175">
    </picture>
</a>
<br>
<br>

[//]: # ( ==== Title ================================================= ) 
# _Perception 3D course software install_

[//]: # ( ==== Hyperlink ============================================= ) 

[//]: # (    <a href="http://132.203.26.125:8111">NorLab TeamCity GUI</a>)
[//]: # (    &#40;VPN/intranet access&#41; &nbsp; • &nbsp;)
<sup>
    <a href="https://github.com/norlab-ulaval">norlab-ulaval</a>
    &nbsp; • &nbsp;
    <a href="https://github.com/norlab-ulaval/percep3d_students">percep3D students</a>
    &nbsp;
</sup>
<br>

[//]: # ( ==== Description =========================================== )
**Software installation scripts and instructions for the Perception 3D robotics perception course**
<br>
<br>
This repository contains installation scripts and detailed instructions for setting up the required <br>
software environment for the Percep3D course. It supports both ROS1 and ROS2 installations on Ubuntu virtual <br>
machines, including all necessary dependencies and configurations for robotics perception exercises.


[//]: # ( ==== Badges ================================================ ) 

[![semantic-release: conventional commits](https://img.shields.io/badge/semantic--release-conventional_commits-453032?logo=semantic-release)](https://github.com/semantic-release/semantic-release)
<img alt="GitHub release (with filter)" src="https://img.shields.io/github/v/release/norlab-ulaval/percep3d_software">

[//]: # (NorLab teamcity)
[//]: # (TODO: Un-comment the next line if your repository has run configuration enable on the norlab-teamcity-server)
[//]: # (<a href="http://132.203.26.125:8111"><img src="https://img.shields.io/static/v1?label=JetBrains TeamCity&message=CI/CD&color=green?style=plastic&logo=teamcity" /></a>)

[//]: # (Dockerhub image badge)
[//]: # (TODO: Un-comment the next line if you have docker images on dockerhub)
[//]: # (TODO: Change "norlabulaval/libpointmatcher" in both url to "your-dockerhub-domain/your-image-name")
[//]: # (<a href="https://hub.docker.com/repository/docker/norlabulaval/libpointmatcher/"> <img alt="Docker Image Version &#40;latest semver&#41;" src="https://img.shields.io/docker/v/norlabulaval/libpointmatcher?logo=docker"> </a>)


[//]: # ( ==== Maintainer ============================================ ) 
<sub>
Maintainer <a href="https://redleader962.github.io">Luc Coupal</a>
</sub>

<br>
<hr style="color:lightgray;background-color:lightgray">
</div>

[//]: # ( ==== Body ================================================== ) 

<details>
  <summary style="font-weight: bolder;font-size: medium;">Table of content</summary>

<!-- TOC -->
* [_Perception 3D course software install_](#_perception-3d-course-software-install_)
  * [Pre-built Virtual Machine (VM)](#pre-built-virtual-machine-vm)
    * [Parallel-Desktop VM](#parallel-desktop-vm-)
    * [VirtualBox VM](#virtualbox-vm)
  * [Instruction for building your own Virtual Machine (VM)](#instruction-for-building-your-own-virtual-machine-vm)
    * [VM provider](#vm-provider)
    * [VM requirement](#vm-requirement)
      * [For running ROS1](#for-running-ros1-)
      * [For running ROS2](#for-running-ros2-)
    * [Ubuntu base images for creating a new VM](#ubuntu-base-images-for-creating-a-new-vm)
      * [Install image for x86 processor:](#install-image-for-x86-processor)
      * [Install image for arm64 processor:](#install-image-for-arm64-processor)
    * [Installation procedure:](#installation-procedure)
      * [Note on install script options:](#note-on-install-script-options)
    * [Software installer script installs steps:](#software-installer-script-installs-steps)
      * [The script `install_percep3d_software_ros1.bash` will execute the following steps:](#the-script-install_percep3d_software_ros1bash-will-execute-the-following-steps)
      * [The script `install_percep3d_software_ros2.bash` will execute the following steps:](#the-script-install_percep3d_software_ros2bash-will-execute-the-following-steps)
  * [Note:](#note-)
    * [To connect remotely to the VM (require the optional ssh server install step)](#to-connect-remotely-to-the-vm-require-the-optional-ssh-server-install-step)
<!-- TOC -->
* [_Instruction for maintainer_](README.maintainer.md#instruction-for-maintainer)

</details>  



## Pre-built Virtual Machine (VM)
Download image and open in your VM provider.
Those VMs come with all the software and course ressources pre-installed. It's the same as spining your own VM, cloning this repository and then executing `install_percep3d_software_ros1.bash`.

### Parallel-Desktop VM 

#### For Apple M chips
- [percep3d-vm-ros1-manual-release.pvmp](https://ulavaldti.sharepoint.com/:u:/s/Percep3D-Organisation/EWofhCPdtFxLuSMPagEa1E0Br8T1bVzY9VxEBY6kyP2rxw?e=QaXrBW)
- [percep3d-vm-ros1-vagrant-release.pvmp](https://ulavaldti.sharepoint.com/:u:/s/Percep3D-Organisation/ERJb-6UeBvhIv1K_xwIJTWUB0tCwWdH_caxzF3LLBxM_1w?e=mLgwyF)

### VirtualBox VM
- See course website

## Instruction for building your own Virtual Machine (VM)

### VM provider
- [Parallels Desktop](https://www.parallels.com/products/desktop/) (Recommended for Mac user. There's a 15 days trial version)
- VirtualBox
- VMware

### VM requirement
Each ROS version is link to Ubuntu distribution. Chose one of the following ubuntu distro:

#### For running ROS1 
- Ubuntu 20.04 (_focal_) for working with ROS1 version _Noetic_
- Ubuntu 18.04 (_bionic_) for working with ROS1 version _Melodic_

#### For running ROS2 
- Ubuntu 24.04 (_noble_) for working with ROS2 version _Jazzy_
- Ubuntu 22.04 (_jammy_) for working with ROS2 version _Humble_
- Ubuntu 20.04 (_focal_) for working with ROS2 version _Foxy_


### Ubuntu base images for creating a new VM
Create a new VM using the menu option "_from an image file_"

#### Install image for x86 processor:
- Ubuntu 20.04 (Focal): [ubuntu-20.04.6-desktop-amd64.iso](https://releases.ubuntu.com/focal/ubuntu-20.04.6-desktop-amd64.iso)
- Ubuntu 22.04 (Focal): [ubuntu-22.04.5-desktop-amd64.iso](https://releases.ubuntu.com/jammy/ubuntu-22.04.5-desktop-amd64.iso)

#### Install image for arm64 processor:

- Ubuntu 20.04 (Focal): [ubuntu-20.04.5-live-server-arm64.iso](https://cdimage.ubuntu.com/releases/20.04/release/ubuntu-20.04.5-live-server-arm64.iso) 
- Ubuntu 22.04 (hammy): [ubuntu-22.04.5-live-server-arm64.iso](https://cdimage.ubuntu.com/releases/22.04/release/ubuntu-22.04.5-live-server-arm64.iso)

---

### Installation procedure:

1. Spin a fresh VM using your prefered virtual machine provider
2. In the VM, open the _terminal_ app, copy the following lines and execute them in the VM terminal <br>
   For installing the ROS1 version:  
   ```shell
   sudo apt-get update && sudo apt-get install --assume-yes git \
     && cd /opt && sudo git clone --recurse-submodules https://github.com/norlab-ulaval/percep3d_software.git \
     && cd percep3d_software/src/vm_software_install_ros1 && sudo bash install_percep3d_software_ros1.bash
   ```
   For installing the ROS2 version:  
    ```shell
    sudo apt-get update && sudo apt-get install --assume-yes git \
      && cd /opt && sudo git clone --recurse-submodules https://github.com/norlab-ulaval/percep3d_software.git \
      && cd percep3d_software/src/vm_software_install_ros2 && sudo bash install_percep3d_software_ros2.bash
    ```
3. Wait for the install script execution end. You will see console message: `Completed install_percep3d_software_ros*.bash`  
4. Logout the current user and login with the new user `student` (password `percep3d`)
5. (For server version `.iso`) Run the following line to install a GUI in the VM
    ```shell
    sudo apt-get install -y ubuntu-desktop && sudo shutdown --reboot now
    ```

#### Note on install script options:
```shell
# Usage:
#   $ bash install_percep3d_software_ros1.bash [--help] [--install-ssh-daemon] [--no-splash]
#
# Arguments:
#   --install-ssh-daemon    Configure and start an ssh daemon on the vm for remote developement
#   --no-splash             Skip the script splash screen (for developer)
#   -h | --help             Script usage with install instruction step reminder
```


---

### Software installer script installs steps:

#### The script `install_percep3d_software_ros1.bash` will execute the following steps:
- Install ROS1 version: _melodic_ or _noetic_
- Install `libpointmatcher` (pined 2022 version) + dependencies (`boost`, `eigen`, `ANN`, `FLANN`, `libnabo`)
- Configure the required directory structure for the course
- Add user `student` with proper privilege
- Fetch the required repository (pined 2022 version) for the _Perception3D_ course and install: 
  - `libpointmatcher_ros`
  - `norlab_icp_mapper`
  - `norlab_icp_mapper_ros`
  - `percep3d_turtle_exercises`
- Fetch ros bag `husky_short_demo.bag`
- Build a `catkin` workspace for ROS
- (Optional) Configure an ssh server (port 2222) for remote VM access 
- Configure a workaround for *rviz* hardware acceleration problem in VM 
- Configure a workaround for running legacy _python2_ ROS tutorial in _python3_ 
- Install *Paraview*


#### The script `install_percep3d_software_ros2.bash` will execute the following steps:
- Install ROS2 version: _Foxy_ or _Humble_ or _Jazzy_
- Install `libpointmatcher` (latest version) + dependencies (`libnabo`, ...)
- Configure the required directory structure for the course
- Add user `student` with proper privilege
- Fetch the required repository (latest version) for the _Perception3D_ course and install: 
  - `libpointmatcher_ros`
  - `norlab_icp_mapper`
  - `norlab_icp_mapper_ros`
  - `percep3d_turtle_exercises`
- Fetch ros bag `husky_short_demo.bag`
- Build a workspace for ROS2
- (Optional) Configure an ssh server (port 2222) for remote VM access 
- Configure a workaround for *rviz* hardware acceleration problem in VM 
- Install *Paraview*


## Note: 

### To connect remotely to the VM (require the optional ssh server install step)
1. first in the VM, open a terminal and execute 
   ```shell
   # Find the VM_IP_ADDRESS using 
   hostname -I | awk '{print $1}'
   ```
2. In the host computer, open a terminal and execute
   ```shell
   # Secure shell
   ssh -p 2222 student@VM_IP_ADDRESS
   
   # or to copy file
   scp -P 2222 </path/to/file> student@VM_IP_ADDRESS:<path/to/dest/dir/>
   ```

