#!/bin/bash
printError(){
    echo -e "ERROR: \e[31m${1}\e[0m"
}
printStep() {
    echo -e "STEP: \e[32m${1}\e[0m"
}
[ `whoami` = root ] || { printError "Run with as root or sudo"; exit 1; }
distro=iron
while getopts ":d:e" opt; do
    case $opt in
        d) 
           distro="${OPTARG}"
        ;;
        e)
            printStep "Install extra software"
            snap install --classic code
            apt update \
            && apt install \
                micro \
                git \
                openssh-server \
            && ufw allow ssh
        ;;
    esac
done

apt update &&  apt install curl gnupg2 lsb-release -y

printStep "Setup language"
locale  # check for UTF-8
apt update &&  apt install locales
locale-gen en_US en_US.UTF-8
update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
locale  # verify settings

printStep "Setup repositories"

apt install software-properties-common -y
if [ $? -gt 0 ]; then
    exit 1;
fi
add-apt-repository universe

curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    |  tee /etc/apt/sources.list.d/ros2.list > /dev/null

curl -sSL https://packages.osrfoundation.org/gazebo.gpg -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
    |  tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

printStep "Install ros dev tools"
apt update && apt install -y ros-dev-tools
if [ $? -gt 0 ]; then
    exit 1;
fi
printStep "Install ros ${distor} Desktop"
apt update && apt install -y ros-${distro}-desktop-full
if [ $? -gt 0 ]; then
    exit 1;
fi
printStep "Verify instalation"
printenv | grep -i ROS

printStep "Set source in .bashrc"
echo -e "source /opt/ros/${ROS_DISTRO}/setup.bash\n" >> ~/.bashrc
cat ~/.bashrc | grep "source /opt/ros/${ROS_DISTRO}/setup.bash" 
if [ $? -gt 0 ]; then
    printError "Missing source command"
fi

printStep "Init rosdep"
rosdep init

printStep "Install colcon"
apt update && apt install python3-colcon-common-extensions -y

printStep "Install basic packages"
apt udpate &&  apt install \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-ros-gazebo-ros-pkgs

apt autoremove && apt clean