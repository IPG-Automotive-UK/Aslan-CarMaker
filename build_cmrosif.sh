# !/bin/bash

#set -v ; # v=verbose (e.g. show comments) x=executions
#set -e ; # exit on error

echo "Execute: ${BASH_SOURCE[0]}";


###   General Settings   ###

ros_ver=1; # ROS Version

dir_init=$(pwd)
dir_script="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
echo "  -> Initial dir = $dir_init"
echo "  -> Script dir  = $dir_script"

# Call this script with argument "all" to enable all parts
if [[ "all" != $@ ]]; then
    # Customize here
    do_ros=1
    do_cmrosif=1
    echo "  -> Run in customized mode!"
else
    # Keep everything enabled!
    do_ros=1
    do_cmrosif=1
fi


###   Help functions   ###
source "${dir_script}/build_help.bash"


###   Build   ###

# ROS specific
# - build external ROS Nodes
# - inclusive shared library for CarMaker ROS Node
if [ $do_ros -eq 1 ] ; then 
    dir_work="ros/ros${ros_ver}_ws"
    echo_head "Build $dir_work"

    cd "$dir_work"
    
    # catkin_make does not return valid value
    # -> checking may fail!
    #cmd_check "./build.sh";
    cmd="./build.sh"; echo "${cmd}"; cmd_echo "${cmd}"
    echo_frmt "  -> Success?\n" y b
    
    cd "$dir_init"
    pwd
fi


# CarMaker executable with CarMaker ROS Interface
# - independent from ROS!!!
# - CarMaker Version for CM executable (see Makefile), CMRosIF library and C
#   CarMaker ROS Node (built in ros workspace) needs to be compatible!!!)
# - needs to be built only once (if no changes in CM Version, CM user modules, ...)
if [ $do_cmrosif -eq 1 ] ; then 
    dir_work="src"
    echo_head "Build $dir_work"
    cmd_check "make -C \"$dir_work\" V=1"
    #cmd_check "make install -C \"$dir_work\" V=1"
fi


###   Finish Script   ###
echo_finish_check
echo_frmt "Building ROS workspace may have failed. Please check Output above!\n" y b

