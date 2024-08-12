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
models=$(ls src/gazebo_model/acto)
for model in $models; do
    echo "Model: $model"
    ln -s $model ~/.gazebo/models/
    export GAZEBO_MODEL_PATH="$GAZEBO_MODEL_PATH:$PWD/src/gazebo_model/acto/$model"
done
${used_shell} ./ros_install_packages -d $distro -p imu-tools -p gps-tools
# configure visual studio code
${used_shell} ./open_vscode