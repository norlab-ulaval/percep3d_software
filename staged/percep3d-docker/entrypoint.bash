#!/bin/bash

#set -e  # exit script if any statement returns a non-true return value

echo
echo -e "Starting container \033[1;37mssh server on port ${D4P3D_SSH_SERVER_PORT}\033[0m with \033[1;37muser ${D4P3D_USER}\033[0m (default pass: ${PASSWORD})"
# sshd flag
# -D : sshd will not detach and does not become a daemon. This allows easy monitoring of sshd.
# -e : sshd will send the output to the standard error instead of the system log.
# -f : config_file
#/usr/sbin/sshd -D -e -f /etc/ssh/sshd_config_ros4percept3d_openssh_server
/usr/sbin/sshd -e -f /etc/ssh/sshd_config_ros4percept3d_openssh_server

echo -e "To connect remotely to the container:
    $ ssh -p ${D4P3D_SSH_SERVER_PORT} ${D4P3D_USER}@$(hostname -I | awk '{print $1}')
    $ sftp -P ${D4P3D_SSH_SERVER_PORT} openssh-$(hostname -I | awk '{print $1}')
    $ scp -P ${D4P3D_SSH_SERVER_PORT} /path/to/foo ${D4P3D_USER}@$(hostname -I | awk '{print $1}'):/dest/
"

#echo -e "Check if ${DS_PYTHON3_VERSION} is working properly by running \033[1;37m\$ python3 /ros_catkin_ws/src/${DS_TARGET_PROJECT_SRC_REPO}/src/container_related/try_pytorch.py\033[0m in the container terminal.
#"


exec "$@"
