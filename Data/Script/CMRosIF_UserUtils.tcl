#
# CMRosIF_UserUtils.tcl
#
# Description:
# - User extension for CarMaker ROS Interface
# - This script is automatically sourced by CMRosIF GUI
#   - Before launch file is started
# - Path for this script and procs can be set in "CMRosIFParameters"
#

namespace eval ::MyCMRosIFUtils {


    #
    # Description:
    # - Hook procedure for CarMaker ROS Interface
    # - The proc is automatically called by the CMRosIF GUI
    # - Needs special parameter configuration in "CMRosIFParameters"
    #   - Cfg.Features = TerminalCmd
    #
    proc LaunchProc {Where InfoDict args} {

        # PopupMsg is only for demonstration!
        # set msg "CMRosIF: User defined Launch at '$Where'\n  \
                   -> InfoDict='$InfoDict'\n  \
                   -> Terminal='[::TerminalCmd getTerminal default]'"
        # PopupMsg info $msg


        # TerminalCmd-API can be used to add multiple Linux Terminals (currently only for Ubuntu)
        # -> Execute "TerminalCmd" in ScriptControl GUI for available commands!

        # Remove the default command parameterized in "CMRosIFParameters" (Launch.args)
        TerminalCmd delTab CMRosIF

        # Add a tab to default Terminal with roscore
        #TerminalCmd addTab "roscore" "source ros_setup.bash; roscore; exec /bin/bash -i"

        # Add a tab to default Terminal
        # TerminalCmd addTab default "source ros_setup.bash; roslaunch --wait hellocm hellocm.launch use_sim_time:=true;exec /bin/bash -i"
	    TerminalCmd addTab "ASLAN" "ros/Aslan/run"

        # Global settings for Terminals
        # Timeout for starting/stopping processes [ms]
        #TerminalCmd config "startwait" 500
        #TerminalCmd config "stopwait"  500

        return 1

    }
}
