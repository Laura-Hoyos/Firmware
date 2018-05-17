#!/bin/bash

#be sure that you ran the gazebo sitl at least once before using the following command:
make posix_sitl_default gazebo_sitl

reg='src\/Firmware$'
if [[ ! "$PWD" =~ $reg ]]; then
	echo "wrong directory, please execute this script in the /src/Firmware directory"
	cd ~/src/Frimware #assuming that PX4 is cloned to that directory
fi

# 0 - Parsing the arguments 

while [[ $# -gt 0 ]]
do
	key="$1"
	case $key in
		-n|--number) #number of simulated drones
		NUMBER="$2"
		shift
		shift
		;;
		-p|--port) #port number for the first drone, then it increments by 10 for each one thereafter
		PORT="$2"
		shift
		shift
		;;
		-ip) #ip of the computer running the gazebo environment
		ipaddress="$2"
		dist=1
		shift
		shift
		;;
		-id) #instance id of the first px4 simulated
		ID="$2"
		shift
		shift
		;;
		-h|--help)
		HELP=1
		shift
		;;
		-nr|--norun)
		norun=1
		shift
		;;
		*) 
		echo "unknown option, please use -h or --help to get more informations"
		exit 1
		;;
	esac
done

if [[ -n $HELP ]]; then
	echo "		#######  help  #######"
	echo ""
	echo "Option -n or --number :"
	echo "	To specify the number of instances of PX4 to be simulated. When not specified, it defaults to 2"
	echo ""
	echo "Option -p or --port :"
	echo "	To fix a port number for the first drone, defauts to 14560"
	echo ""
	echo "Option -ip : "
	echo "	If you want to run the instances of PX4 remotly, you need to specfify the ip address of the computer running the siulation environment with this option (can be obtained by running the command 'hostname -I'), as well as the ID of the first instance to be simulated in this execution with the option -id"
	echo ""
	echo "to only generate the scripts without executing them, use the option -nr (no run)"
	exit 1
fi

if [[ -z $NUMBER ]]; then #if the number of simulated drones is not defined, it defaults to 2
	NUMBER=2 
fi

if [[ -z $PORT ]]; then #if the port number for the first drone is not specified, it defaults to 14560
	PORT=14560 
fi

if [[ -n $ipaddress  && -z $ID ]]; then
	echo "If you want to run an instance of PX4 remotely, please provide an ID with the option -id"
	exit 1
fi

if [[ -z $ipaddress ]]; then #if the ip address is not defined, it defaults to the local ip
	#ipaddress=$(ip route get 8.8.8.8 | awk '{print $NF; exit}')
	ipaddress=$(hostname -I)
fi

if [[ -z $ID ]]; then #if the ID is not defined, default id of the first pixhawk simulated is 1
	ID=1;
fi
