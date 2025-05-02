# Percep3D course software install 
Maintainer: luc.coupal.1@ulaval.ca

## Percep3D for Virtual Machine (VM)
#### VM requirement
- Ubuntu 20.04 (Focal) will install ROS version Noetic
- Ubuntu 18.04 (Bionic) will install ROS version Melodic
- (Mac user) Recommend using [Parallels Desktop](https://www.parallels.com/products/desktop/) (there's a 30 day trial version)

#### Script usage:
1. Open the VM
2. In the VM, open the _terminal_ app, copy the following lines and execute them in the VM terminal
    ```shell
    sudo apt-get update && sudo apt-get install --assume-yes git \
      && cd /opt && sudo git clone https://github.com/norlab-ulaval/percep3d_software.git \
      && cd percep3d_software/percep3d-VM-software \
      && sudo bash install_percept3d_software.bash
    ```
3. Logout current user and login with user `student` pass `percept3d`

#### Software installs step:
The script `install_percept3d_software.bash` will execute the following steps:
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
   scp -P 2222 /path/to/foo student@VM_IP_ADDRESS:/dest/
   ```


#### Unit-test execution step on aarch arm64 (Apple M1 chips): 
```shell
docker pull --platform linux/arm64 ubuntu:20.04
docker build --platform linux/arm64 -f Dockerfile.test -t percep3d-vm-software-tester-ubuntu:20.04 . 
docker run -a --name iAmTestROSmelodic4vmContainer -t -i percep3d-vm-software-tester-ubuntu:20.04 
```

---

## Percep3D in Docker (In progress)
Build either 
- Ubuntu 20.04 (Focal) with ROS version Noetic
- Ubuntu 18.04 (Bionic) with ROS version Melodic


