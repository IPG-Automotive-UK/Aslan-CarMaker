# !/bin/bash

#
# Author: IPG Automotive/fs
# Date:   13.04.18
# 
# Description:
# - Method according to "Catkin-y approach" on http://wiki.ros.org/IDEs
# - Prepare a ROS catkin workspace for eclipse
# - Eclipse project files are added to the build/ folder in ROS catkin workspace
# - Current shell environment is added to the make process in eclipse
#
# Usage:
# - Copy this script into a valid catkin workspace with existing and comilablle ROS packages
# - After execution of this script
#   - Create a new eclipse workspace (e.g. directly in the catkin workspace or outside)
#   - start eclipse and import the eclipse project located in the "build/" folder
# 
# Troubleshooting:
# - No compilation in eclipse possible (e.g. eclipse console output is empty)
#   -> Problem was observed with a quite new eclipse version
#   -> Old catkin_make or ROS?
#      -> Upgrade you ROS
#      -> e.g. via "sudo apt-get update; sudo apt-get upgrade ros-kinetic*"
#   -> Import project into eclipse workspace with older eclipse version 
#
# - Preparation might fail if folders "build", "install", "devel" already exits
#   - e.g. build not possible inside eclipse
#   - in this case delete the folders and execute this script again
#

echo "Preparation of ROS catkin workspace for eclipse started..."

cmd="pwd"; echo "Current working dir: $cmd"; $cmd
cmd="source /opt/ros/ros1/setup.bash"; echo $cmd; $cmd

dirs2del="build/ devel/ install/"

echo ""
echo -n "Delete '$dirs2del'? [y/n] "

read rv

if [ "$(echo "$rv" | grep -i "^y")" ] ; then
    cmd="rm -r ${dirs2del}"; echo $cmd; $cmd
fi

echo ""
echo "Build ROS workspace for eclipse..."
catkin_make --force-cmake -G"Eclipse CDT4 - Unix Makefiles"

echo "Pass the current shell environment into the make process in eclipse..."
awk -f $(rospack find mk)/eclipse.awk build/.project > build/.project_with_env && mv build/.project_with_env build/.project


echo "Preparation of ROS catkin workspace for eclipse finished!"

