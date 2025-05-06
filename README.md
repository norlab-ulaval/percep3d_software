# Percep3D course software install 
Maintainer: luc.coupal.1@ulaval.ca

---


**Table of content**
<!-- TOC -->
* [Percep3D course software install](#percep3d-course-software-install-)
  * [Percep3D course software install, for Virtual Machine (VM)](#percep3d-course-software-install-for-virtual-machine-vm)
    * [VM requirement for ROS1](#vm-requirement-for-ros1)
    * [VM requirement for ROS2](#vm-requirement-for-ros2)
    * [VM provider](#vm-provider)
    * [Ubuntu install images](#ubuntu-install-images)
      * [Install image for x86 processor](#install-image-for-x86-processor)
      * [Install image for arm64 processor](#install-image-for-arm64-processor)
    * [Script usage:](#script-usage)
    * [Software installs step:](#software-installs-step)
      * [`install_percep3d_software_ros1.bash` ](#the-script-install_percep3d_software_ros1bash-will-execute-the-following-steps)
      * [`install_percep3d_software_ros2.bash` ](#the-script-install_percep3d_software_ros2bash-will-execute-the-following-steps)
  * [Note:](#note-)
    * [To connect remotely to the VM (require the optional ssh server install step)](#to-connect-remotely-to-the-vm-require-the-optional-ssh-server-install-step)
* [Instruction for maintainer](README.dev.md#development-resources)
<!-- TOC -->

---

## Percep3D course software install, for Virtual Machine (VM)

### VM requirement for ROS1
One of the following ubuntu distro:
- Ubuntu 20.04 (_focal_) for working with ROS1 version _Noetic_
- Ubuntu 18.04 (_bionic_) for working with ROS1 version _Melodic_

### VM requirement for ROS2
One of the following ubuntu distro:
- Ubuntu 24.04 (_noble_) for working with ROS2 version _Jazzy_
- Ubuntu 22.04 (_jammy_) for working with ROS2 version _Humble_
- Ubuntu 20.04 (_focal_) for working with ROS2 version _Foxy_

### VM provider
- [Parallels Desktop](https://www.parallels.com/products/desktop/) (Recommended for Mac user. There's a 30 day trial version)
- VirtualBox
- VMware

### Ubuntu install images
#### Install image for x86 processor:
- Ubuntu 20.04 (Focal): [ubuntu-20.04.6-desktop-amd64.iso](https://releases.ubuntu.com/focal/ubuntu-20.04.6-desktop-amd64.iso)
- Ubuntu 22.04 (Focal): [ubuntu-22.04.5-desktop-amd64.iso](https://releases.ubuntu.com/jammy/ubuntu-22.04.5-desktop-amd64.iso)

#### Install image for arm64 processor:
**Note(arm64 version)**: execute `sudo apt-get install -y ubuntu-desktop && sudo shutdown --reboot now` on vm first spin. 
- Ubuntu 20.04 (Focal): [ubuntu-20.04.5-live-server-arm64.iso](https://cdimage.ubuntu.com/releases/20.04/release/ubuntu-20.04.5-live-server-arm64.iso) 
- Ubuntu 22.04 (hammy): [ubuntu-22.04.5-live-server-arm64.iso](https://cdimage.ubuntu.com/releases/22.04/release/ubuntu-22.04.5-live-server-arm64.iso)
---

### Script usage:
1. Open the VM
2. In the VM, open the _terminal_ app, copy the following lines and execute them in the VM terminal
   - For installing the ROS1 version:  
       ```shell
       sudo apt-get update && sudo apt-get install --assume-yes git \
         && cd /opt && sudo git clone --recurse-submodules https://github.com/norlab-ulaval/percep3d_software.git \
         && cd percep3d_software/vm_software_install_ros1 && sudo bash install_percep3d_software_ros1.bash
       ```
   - For installing the ROS2 version:  
        ```shell
        sudo apt-get update && sudo apt-get install --assume-yes git \
          && cd /opt && sudo git clone --recurse-submodules https://github.com/norlab-ulaval/percep3d_software.git \
          && cd percep3d_software/vm_software_install_ros2 && sudo bash install_percep3d_software_ros2.bash
        ```
3. Wait for the install script execution end. You will see console message: `Completed install_percep3d_software_ros*.bash`  
4. Logout the current user and login with the new user `student` (password `percep3d`)
5. (optional) If you're using a server version `.iso`, just run the following line to install a GUI
    ```shell
    sudo apt-get install --assume-yes --no-install-recommends ubuntu-desktop
    sudo shutdown --reboot now
    ```

---

### Software installs step:

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

