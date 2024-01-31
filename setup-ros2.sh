#!/bin/bash
printError(){
    echo -e "ERROR: \e[31m${1}\e[0m"
}
printStep() {
    echo -e "STEP: \e[32m${1}\e[0m"
}

printInput(){
    echo -e "INPUT: \e[32m${1}\e[0m"
}

setupGit(){
    printStep "Setup git config"
    printInput "Git user name"
    read userName
    git config --global user.name "$userName"
    printInput "Git email"
    read email
    git config --global user.email "$email"
}

distro=iron
while getopts ":d:esg" opt; do
    case $opt in
        d) 
           distro="${OPTARG}"
        ;;
        e)
            printStep "Install vscode extensions"
            code --help > /dev/null
            if [ $? -ne 0 ]; then
            	 sudo snap install --classic code 
			fi
           	code --install-extension .vscode/extensions.json
            exit $?
        ;;
        s) 
            sudo apt update && sudo apt install openssh-server && sudo ufw allow ssh
            exit $?
        ;;
    esac
done

setupGit

printStep "Install basic"
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
echo -e "source /opt/ros/${distro}/setup.bash\n" >> ~/.bashrc
cat ~/.bashrc | grep "source /opt/ros/${distro}/setup.bash" 

if [ $? -gt 0 ]; then
    printError "Missing source command"
fi
source /opt/ros/${distro}/setup.bash
printStep "Init rosdep"
sudo rosdep init
rosdep update

printStep "Install colcon"
sudo apt update && sudo apt install python3-colcon-common-extensions -y

printStep "Install basic packages"
sudo apt update && sudo apt install -y \
    ros-${distro}-joint-state-publisher \
    ros-${distro}-joint-state-publisher-gui \
    ros-${distro}-xacro \
    ros-${distro}-gazebo-ros-pkgs

sudo apt autoremove && sudo apt clean

code --help > /dev/null

if [ $? -ne 0 ]; then
	sudo snap install --classic code
fi
code --install-extension .vscode/extensions.json
