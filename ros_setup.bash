#
# Script to prepare environment for CarMaker ROS Interface
#
# Description:
# - Prepare current environment for ROS usage
# - Source this script via terminal or e.g add content to your ~/.bashrc
# - Use "source <this script>", or ". <this script>"
# - Executing this script makes sourcing below available in child processes
# - Used paths are expected relative to location of this script file
#
# ToDo:
# - Error handling for 
#   - Do not use shell's "set -e"! Error handling with sourcing scripts may fail
#
# Creator: IPG Automotive GmbH/fs
# 

echo "Execute: ${BASH_SOURCE[0]}";


# General Settings
_str="ros1";
_dir_script="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"; 


###   Help functions   ###

# Echo and call command
# - e.g. _cmd_echo "pwd"
# - e.g. _cmd_warn "pwd" "Be careful when printing the current working directory!"
_cmd_echo() { echo "  -> run '${1}'"; eval ${1}; }
_cmd_warn() { echo -e "\033[1;31m  Warning:${2}\n  -> run '${1}'\033[0m"; eval ${1}; }


###   ROS preparation   ###

# Additional libraray paths (e.g. for debugging)
#_cmd_warn "export LD_LIBRARY_PATH=\"$(cd \"${_dir_script}/src_${_str}\"; pwd)\"" " Overwrite global variable"

# Global ROS installation
#_cmd_echo "source \"/opt/ros/${_str}/setup.bash\""

# Local ROS workspace
_cmd_echo "source \"${_dir_script}/ros/${_str}_ws/devel/setup.bash\""
#_cmd_echo "source \"${_dir_script}/ros/${_str}_ws/install/setup.bash\""

echo "  -> LD_LIBRARY_PATH=${LD_LIBRARY_PATH}"


# Cleanup (You may have been working directly inside a shell environment!)
unset _str _dir_script
unset -f _cmd_echo _cmd_warn

