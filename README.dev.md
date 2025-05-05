# Instruction for maintainer

---

## Development workflow in virtual machine (VM) with _Vagrant_ 

The `Vagrantfile` at repository root is custom-made with the following configuration
- the repository codebase is unidirectionally synchronized in the VM 
- auto `rsync` on `vagrant up`;
- auto `rsync` on `vagrant snapshot restore`;
- the VM is provisioned with all the necessary developement tools

### Initial setup
1. Install [Vagrant](https://www.vagrantup.com) 
2. Install a provider ([parallels desktop + Vagrant](http://parallels.github.io/vagrant-parallels/docs/) on Mac OsX). Dont forget to modify the `Vagrantfile` accordingly if you use a different provider.

### To start the virtual machine
```bash
# execute in root
$ vagrant up
$ vagrant ssh
$ cd /opt/percep3d_software
# ... et voila
```
Note: `vagrant ssh` will log in the VM default username `vagrant`. To directly log with the TeamCity Server admin username once its created simply use ssh:  
```bash
$ ssh -o StrictHostKeyChecking=no $ADMIN_USER@$SERVER_IP

$ ssh -o StrictHostKeyChecking=no student@132.203.26.125
```
`-o StrictHostKeyChecking=no` is a trick to enable using the same IP address as the real server and


### To save a snapshot in time
```bash
$ vagrant snapshot save --force myCoolSnashotName
$ vagrant snapshot restore myCoolSnashotName
```

### To update your VM
```bash
# To manually sync your local repository codebase to the virtual machine (unidirectional sync)
$ vagrant rsync

# To update your Vagrantfile definition in the VM
$ vagrant reload
```

### When your done
```bash
# To stop the VM
$ vagrant halt

# or remove your VM
$ vagrant destroy
```


# Unit-test execution step on aarch arm64 (Apple M1 chips): 
```shell
docker pull --platform linux/arm64 ubuntu:20.04
docker build --platform linux/arm64 -f Dockerfile.test -t percep3d-vm-software-tester-ros1-ubuntu:20.04 . 
docker run -a --name IAmPercep3D -t -i percep3d-vm-software-tester-ros1-ubuntu:20.04 
```

