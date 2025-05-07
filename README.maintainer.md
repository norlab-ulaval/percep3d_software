# Instruction for maintainer

---

## Unit-test execution
```shell
# Unit-test ros1 installer
bash tests/test_install_percep3d_software_ros1.bash

# Unit-test ros2 installer
bash tests/test_install_percep3d_software_ros2.bash
```

---

## Development workflow in virtual machine (VM) with _Vagrant_ 

The `Vagrantfile` at repository root is custom-made with the following configuration
- the repository codebase is unidirectionally synchronized in the VM 
- auto `rsync` on `vagrant up`;
- auto `rsync` on `vagrant snapshot restore`;
- the VM is provisioned with all the necessary developement tools

### Initial setup
1. Install [Vagrant](https://www.vagrantup.com). Run `vagrant --help` to check available command. 
2. Install a provider. Dont forget to modify the `Vagrantfile` accordingly to your choosen provider.
   - [parallels desktop + Vagrant](http://parallels.github.io/vagrant-parallels/docs/) on Mac OsX 
   - [VirtualBox](https://developer.hashicorp.com/vagrant/docs/providers/virtualbox)
   - [VMware](https://developer.hashicorp.com/vagrant/docs/providers/vmware)
   - [Docker](https://developer.hashicorp.com/vagrant/docs/providers/docker)

### To start the virtual machine
```bash
# Example for the ROS2 vm
$ cd path/to/percep3d_software/src/vm_software_install_ros2/

# Be sure to always run vagrant command from the directory where the Vagrantfile is.
$ vagrant up
```
Note: `vagrant up` is configured to automaticaly rsync the vm content

### To `ssh` the virtual machine
```bash
$ vagrant ssh
# vagrant@percep3d:~$

$ cd /opt/percep3d_software/src/vm_software_install_ros2
# vagrant@percep3d:/opt/percep3d_software/src/vm_software_install_ros2$

$ sudo bash install_percep3d_software_ros2.bash
# ... et voila
```
#### Note: 
- The VM hostname is `percep3d`
- The VM is configured with a static IP address: `132.203.26.125`
- `vagrant ssh` will login the VM box default user `vagrant`
 
#### To login with the percep3d course script created new user `student`:  
```bash
$ ssh -o StrictHostKeyChecking=no student@132.203.26.125
# -o StrictHostKeyChecking=no is a trick to enable using the same IP address as the real server and
```


### To save a snapshot in time
```bash
# To save a VM state in time
$ vagrant snapshot save --force <myCoolSnashotName>

# To restore a saved snapshot
$ vagrant snapshot restore <myCoolSnashotName>
```
Note: `vagrant snapshot restore` is configured to automaticaly rsync the vm content

### To update your VM
```bash
# To manually sync your local repository codebase to the virtual machine (unidirectional sync)
$ vagrant rsync

# To update your Vagrantfile definition in the VM
$ vagrant provision  

# To update your Vagrantfile definition in the VM
$ vagrant reload
```

### When your done
```bash
# To stop the VM
$ vagrant halt

# or remove your VM and check vm status
$ sudo vagrant destroy --force && vagrant global-status --prune
```



## Development in docker container  
**Note**: execute command from repository root

### Host aarch

#### ROS1 vm:
```shell
docker pull ubuntu:20.04
docker build \
        --build-arg BASE_IMAGE=ubuntu:20.04 \
        -f src/vm_software_install_ros1/Dockerfile.test \
        -t perce3d-software-ros1:noetic-full-ubuntu-20.04 \
        . 
docker run --name IamPercep3D-Noetic --init --rm -it perce3d-software-ros1:noetic-full-ubuntu-20.04 
```

#### ROS2 vm
```shell
docker pull ubuntu:22.04
docker build \
        --build-arg BASE_IMAGE=ubuntu:22.04 \
        -f src/vm_software_install_ros2/Dockerfile.test \
        -t perce3d-software-ros2:humble-full-ubuntu-22.04 \
        .
docker run --name IamPercep3D-Humble --init --rm -it perce3d-software-ros2:humble-full-ubuntu-22.04 
```

### arm64 specific (e.g. Apple M1 chips)

#### ROS1 vm
```shell
docker pull --platform linux/arm64 ubuntu:20.04
docker build \
        --build-arg BASE_IMAGE=ubuntu:20.04 \
        --platform linux/arm64 \
        -f src/vm_software_install_ros1/Dockerfile.test \
        -t perce3d-software-ros1:noetic-full-ubuntu-20.04 \
        . 
docker run --name IamPercep3D-Noetic --init --rm -it perce3d-software-ros1:noetic-full-ubuntu-20.04 
```


#### ROS2 vm
```shell
docker pull --platform linux/arm64 ubuntu:22.04
docker build \
        --build-arg BASE_IMAGE=ubuntu:22.04 \
        --platform linux/arm64 \
        -f src/vm_software_install_ros2/Dockerfile.test \
        -t perce3d-software-ros2:humble-full-ubuntu-22.04 \
        .
docker run --name IamPercep3D-Humble --init --rm -it perce3d-software-ros2:humble-full-ubuntu-22.04 
```

