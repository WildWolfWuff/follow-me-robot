used_shell=${SHELL##*/}
echo "Use shell: ${used_shell}"
. ./.setup/func/func.${used_shell}

distro=humble
while getopts ":d:sh" opt; do
    case $opt in
        d) 
           distro="${OPTARG}"
        ;;
        s) 
            sudo apt update && sudo apt install openssh-server && sudo ufw allow ssh
            exit $?
        ;;
        ?|h)
            printText "Usage: $(basename $0) [-d ros distro] [-s] [-h]"
            exit 1
        ;;
    esac
done
sudo echo "Start setup ros2 for $distro"
cd .setup
# setup git
${used_shell} ./git_config

# install ros system
${used_shell} ./ros_install -d $distro -tf
if [ $? -gt 0 ]; then
    exit 1;
fi

# add ros script to shell configuration
${used_shell} ./ros_add_source -d $distro
if [ $? -gt 0 ]; then
    exit 1;
fi

# intialize ros project
${used_shell} ./ros_init -d $distro
if [ $? -gt 0 ]; then
    exit 1;
fi

mkdir -p ~/.gazebo/models
if [ ! -d "~/.gazebo/models/hospital" ]; then
    ln -s $PWD/src/worlds/hospital ~/.gazebo/models/
    ln -s $PWD/src/gazebo_model/april_tag0 ~/.gazebo/models/
    ln -s $PWD/src/gazebo_model/april_tag1 ~/.gazebo/models/
    ln -s $PWD/src/gazebo_model/april_tag2 ~/.gazebo/models/
    ln -s $PWD/src/gazebo_model/april_tag3 ~/.gazebo/models/
fi

${used_shell} ./ros_install_packages -d $distro -p imu-tools -p gps-tools 
# configure visual studio code
${used_shell} ./open_vscode