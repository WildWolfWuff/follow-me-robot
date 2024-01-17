#!/bin/bash
printError(){
    echo -e "ERROR: \e[31m${1}\e[0m"
}
printStep() {
    echo -e "STEP: \e[32m${1}\e[0m"
}

distro=iron
while getopts ":d:e" opt; do
    case $opt in
        d) 
           distro="${OPTARG}"
        ;;
        e)
            printStep "Install extra software"
            sudo snap install --classic code
            sudo apt update \
            && sudo apt install \
                micro \
                git \
                openssh-server \
            && sudo ufw allow ssh
        ;;
    esac
done

sudo apt update && sudo apt install curl gnupg2 lsb-release -y

printStep "Setup language"
locale  # check for UTF-8
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
locale  # verify settings

printStep "Setup repositories"
sudo apt install software-properties-common -y
if [ $? -gt 0 ]; then
    exit 1;
fi
sudo add-apt-repository universe

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
sudo echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo curl -sSL https://packages.osrfoundation.org/gazebo.gpg -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
sudo echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
    | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

printStep "Install ros dev tools"
sudo apt update && sudo apt install -y ros-dev-tools
if [ $? -gt 0 ]; then
    exit 1;
fi
printStep "Install ros ${distor} Desktop"
sudo apt update && sudo  apt install -y ros-${distro}-desktop-full
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
sudo rosdep init
rosdep update

printStep "Install colcon"
sudo apt update && sudo apt install python3-colcon-common-extensions -y

printStep "Install basic packages"
sudo apt update && sudo apt install \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-ros-gazebo-ros-pkgs

sudo apt autoremove && sudo apt clean