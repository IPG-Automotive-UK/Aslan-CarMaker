# !/bin/bash

#set -v ; # v=verbose (e.g. show comments) x=executions
#set -e ; # exit on error

echo "Execute: ${BASH_SOURCE[0]}";

nerr=0 ; # number of errors for this script

###   Help functions   ###
# If used in "source" script
# - Environment variables/functions are direclty affected!
#   - e.g. already existing will be overwritten, new will be available after script finished
# - Use naming "_<variable/function>" 
#   - e.g. _dir_script="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)";
# - Unset before exit
#   - e.g. variables: unset _dir_script _str
#   - e.g. functions: unset -f _cmd_echo 

# Check echo color/style 
echo_demo() {
   for ((k=0;k<10;k++)); do
        for (( i = 0; i < 50; i++ )); do
            echo -e "Result for k=${k};k=${i} = \033[${k};${i}m Hello World!\033[0m"
        done
    done
}


# Echo and call command
# - e.g. cmd_echo "pwd"
# - e.g. cmd_warn "pwd" "Be careful when printing the current working directory!"
cmd_echo() { echo "  -> run '${1}'"; eval ${1}; }
cmd_warn() { echo -e "\033[1;33m  Warning:${2}\n  -> run '${1}'\033[0m"; eval ${1}; }


# Echo, call command and check if error occured!
cmd_check() { echo "${1}"; cmd_echo "${1}"; echo_cmdcheck; }


# Echo exit status of the most recently command
# - Must be immediately called after command to be checked,
#   otherwise next command will overwrite $? variable
# - Does not exexute the command
echo_cmdcheck() {
    if [ $? -eq 0 ]
        then echo_frmt "$1  -> Success\n" g b
        else echo_frmt "$1  -> Error\n" r b; ((nerr++))
    fi        
}


# Echo for frequently used format/highlighting
# - color and style is optional
# - e.g. "echo_frmt echo_head <Text> ?<r(ed), g(reen), b(lue), ...>? ? 
echo_frmt() {
    b=0; c=0
    case $2 in 
        r*) c=31;;
        g*) c=32;;
        y*) c=33;;
        b*) c=34;;
    esac

    case $3 in
        b*) b=1;;
    esac

    echo -e "\033["$b";"$c"m$1\033[0m"
}


# Echo text as "headline" in optional color
# - e.g. echo_head <Text> ?<r(ed), g(reen), b(lue), ...>?
echo_head() {
    if [ $# -eq 1 ]
        then c="b"
        else c=$2
    fi
    echo_frmt "\n###   $1   ###\n" $c b
}


# Final check if everything worked as expected
echo_finish_check() {
    if [ $nerr -eq 0 ]
        then echo_head "Script '$0' finished successfully!" g
        else echo_head "Script '$0' finished with $nerr Error(s)!" r
    fi
}





