# Percep3D course software install 
Maintainer: luc.coupal.1@ulaval.ca

---

<!-- TOC -->
* [Percep3D course software install](#percep3d-course-software-install-)
  * [Percep3D for Virtual Machine (VM)](#percep3d-for-virtual-machine-vm)
      * [VM requirement](#vm-requirement)
      * [Script usage:](#script-usage)
      * [Software installs step:](#software-installs-step)
    * [Note:](#note-)
      * [To connect remotely to the VM](#to-connect-remotely-to-the-vm)
  * [Percep3D in Docker (In progress)](#percep3d-in-docker-in-progress)
* [Instruction for maintainer](README.dev.md#development-resources)
<!-- TOC -->

---

## Percep3D for Virtual Machine (VM)
#### VM requirement
One of the following ubuntu distro:
- Ubuntu 20.04 (Focal) for working with ROS1 version Noetic
- Ubuntu 18.04 (Bionic) for working with ROS1 version Melodic

#### VM provider
- (Note for Mac user) Recommend using [Parallels Desktop](https://www.parallels.com/products/desktop/) (there's a 30 day trial version)

#### Script usage:
1. Open the VM
2. In the VM, open the _terminal_ app, copy the following lines and execute them in the VM terminal
    ```shell
    sudo apt-get update && sudo apt-get install --assume-yes git \
      && cd /opt && sudo git clone --recurse-submodules https://github.com/norlab-ulaval/percep3d_software.git \
      && cd percep3d_software/vm_software_install_ros1 && sudo bash install_percep3d_software_ros1.bash
    ```
3. Logout current user and login with user `student` pass `percep3d`

#### Software installs step:
The script `install_percep3d_software_ros1.bash` will execute the following steps:
- Install ROS version: _melodic_ or _noetic_
- Install `libpointmatcher` (latest) + dependencies (boost, eigen, ANN, FLANN, libnabo)
- Configure the required directory structure for the course
- Add user `student` with proper privilege
- Fetch the required repository for the _Perception3D_ course and install: 
  - `libpointmatcher_ros`
  - `norlab_icp_mapper`
  - `norlab_icp_mapper_ros`
  - `percep3d_turtle_exercises`
- Fetch ros bag `husky_short_demo.bag`
- Build a catkin workspace for ROS
- Configure an ssh server for remote VM access
- Configure a workaround for *rviz* hardware acceleration problem in VM 
- Configure a workaround for running legacy _python2_ ROS tutorial in _python3_ 
- Install *Paraview*


### Note: 

#### To connect remotely to the VM
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

