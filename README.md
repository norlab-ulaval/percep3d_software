# ROS-4-Percept3D course software install 
Maintainer: luc.coupal.1@ulaval.ca

## ROS-4-Percept3D for virtual machine
#### Minimum VM requirement
- 20.04 <= Ubuntu version
- Tested on Ubuntu 20.04 (Focal)
- (Mac user) Recommand using [Parallels Desktop](https://www.parallels.com/products/desktop/) (there's a 30 day trial version)

#### Script usage:
1. In the VM, execute the following line in a terminal
   ```shell
   cd /opt \
      && sudo apt-get update \
      && sudo apt-get install --assume-yes git \
      && sudo git clone https://github.com/norlab-ulaval/dockerized-ROS4percept3D.git \
      && cd dockerized-ROS4percept3D/ros-percept3D-4-VM \
      && sudo bash install_percept3d.bash
   ```
2. logout current user and login with user `student` pass `percept3d`

#### Script install step: 
- Install ROS version: melodic
- Install `libpointmatcher` (latest) + dependencies (boost, eigen, ANN, FLANN, libnabo)
- Configure the required directory structure for the course
- Add user `student` with proper privilege
- Fetch the required repository for the _Perception3D_ course and install: 
  - libpointmatcher_ros
  - norlab_icp_mapper
  - norlab_icp_mapper_ros
  - percep3d_turtle_exercises
- Fetch ros bag `husky_short_demo.bag`
- Build a catkin workspace 
- Configure an ssh server for remote VM access
- Workaround for rviz hardware acceleration problem in VM 
- Workaround for running legacy python2 ROS tutorial in python3 
- Install Paraview


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
docker build --platform linux/arm64 -f Dockerfile -t test-percept3d4vm-ubuntu:20.04 . 
docker run -a --name iAmTestROSmelodic4vmContainer -t -i test-percept3d4vm-ubuntu:20.04 
```

---

## ROS-4-Percept3D in Docker (In progress)
Build either 
- Ubuntu 20.04 (Focal) with ROS version Noetic
- Ubuntu 18.04 (Bionic) with ROS version Melodic


