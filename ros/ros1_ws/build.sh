# !/bin/bash

echo "Run catkin build"

cmd="source /opt/ros/ros1/setup.bash"; echo $cmd; $cmd
cmd="catkin_make"; echo $cmd; $cmd
#cmd="catkin_make install"; echo $cmd; $cmd

# ToDo:
# - catkin may not manipulate exit status
#   - even if error occurs, script may continue without error...
#     -> e.g. when compiling msg with unknown datatype
if [ $? -ne 0 ]
    then exit $?
fi

echo "Info: Execute 'source ./devel/setup.bash' to prepare your environment!"
