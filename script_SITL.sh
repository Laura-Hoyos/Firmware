#!/bin/bash

#be sure that you ran the gazebo sitl at least once before using the following command:
#make posix_sitl_default gazebo_sitl

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

PORT=$((PORT+(ID-1)*10))

date_u=$(date +%s)
if [ ! -d simulation_$date_u ]; then
	mkdir simulation_$date_u
fi

# Creating conf files .urdf and launch files for each drone

for ((i=ID; i<=$((NUMBER+ID-1)); i++))
do
	mavport=$((PORT-4))
	mavport2=$((PORT-5))
	mavoport=$((PORT-2))
	echo "port drone "$i" : "$PORT
	echo "mavport "$i" : "$mavport
	echo "mavport2 "$i" : "$mavport2
	echo "mavoport "$i" : "$mavoport
	/opt/ros/kinetic/share/xacro/xacro.py $(pwd)/Tools/sitl_gazebo/models/rotors_description/urdf/iris_base.xacro rotors_description_dir:=$(pwd)/Tools/sitl_gazebo/models/rotors_description mavlink_udp_port:=$PORT > simulation_$date_u/iris_$i.urdf
	echo 'uorb start
	param load
	dataman start
	param set MAV_SYS_ID '$i'
	param set BAT_N_CELLS 3
	param set CAL_ACC0_ID 1376264
	param set CAL_ACC0_XOFF 0.01
	param set CAL_ACC0_XSCALE 1.01
	param set CAL_ACC0_YOFF -0.01
	param set CAL_ACC0_YSCALE 1.01
	param set CAL_ACC0_ZOFF 0.01
	param set CAL_ACC0_ZSCALE 1.01
	param set CAL_ACC1_ID 1310728
	param set CAL_ACC1_XOFF 0.01
	param set CAL_GYRO0_ID 2293768
	param set CAL_GYRO0_XOFF 0.01
	param set CAL_MAG0_ID 196616
	param set CAL_MAG0_XOFF 0.01
	param set COM_DISARM_LAND 3
	param set COM_OBL_ACT 2
	param set COM_OBL_RC_ACT 0
	param set COM_OF_LOSS_T 5
	param set COM_RC_IN_MODE 1
	param set EKF2_AID_MASK 1
	param set EKF2_ANGERR_INIT 0.01
	param set EKF2_GBIAS_INIT 0.01
	param set EKF2_HGT_MODE 0
	param set EKF2_MAG_TYPE 1
	param set MAV_TYPE 2
	param set MC_PITCH_P 6
	param set MC_PITCHRATE_P 0.2
	param set MC_ROLL_P 6
	param set MC_ROLLRATE_P 0.2
	param set MIS_TAKEOFF_ALT 2.5
	param set MPC_HOLD_MAX_Z 2.0
	param set MPC_Z_VEL_I 0.15
	param set MPC_Z_VEL_P 0.6
	param set NAV_ACC_RAD 2.0
	param set NAV_DLL_ACT 2
	param set RTL_DESCEND_ALT 5.0
	param set RTL_LAND_DELAY 5
	param set RTL_RETURN_ALT 30.0
	param set SENS_BOARD_ROT 0
	param set SENS_BOARD_X_OFF 0.000001
	param set SYS_AUTOSTART 4010
	param set SYS_MC_EST_GROUP 2
	param set SYS_RESTART_TYPE 2
	param set SITL_UDP_PRT '$PORT'
	param set MAV_BROADCAST 1
	replay tryapplyparams
	simulator start -s
	tone_alarm start
	gyrosim start
	accelsim start
	barosim start
	adcsim start
	gpssim start
	pwm_out_sim mode_pwm
	sensors start
	commander start
	land_detector start multicopter
	navigator start
	ekf2 start
	mc_pos_control start
	mc_att_control start
	mixer load /dev/pwm_output0 ROMFS/px4fmu_common/mixers/quad_dc.main.mix
	mavlink start -x -u '$mavport' -r 4000000
	mavlink start -x -u '$mavport2' -r 4000000 -m onboard -o '$mavoport'
	mavlink stream -r 50 -s POSITION_TARGET_LOCAL_NED -u '$mavport'
	mavlink stream -r 50 -s LOCAL_POSITION_NED -u '$mavport'
	mavlink stream -r 50 -s GLOBAL_POSITION_INT -u '$mavport'
	mavlink stream -r 50 -s ATTITUDE -u '$mavport'
	mavlink stream -r 50 -s ATTITUDE_QUATERNION -u '$mavport'
	mavlink stream -r 50 -s ATTITUDE_TARGET -u '$mavport'
	mavlink stream -r 50 -s SERVO_OUTPUT_RAW_0 -u '$mavport'
	mavlink stream -r 20 -s RC_CHANNELS -u '$mavport'
	mavlink stream -r 250 -s HIGHRES_IMU -u '$mavport'
	mavlink stream -r 10 -s OPTICAL_FLOW_RAD -u '$mavport'
	logger start -e -t
	mavlink boot_complete
	replay trystart'>>simulation_$date_u/iris_$i

	echo '<launch>
    <arg name="est" default="ekf2"/>
    <arg name="vehicle" default="iris"/>
    <arg name="ns" default="/"/>
    <arg name="pluginlists_yaml" default="$(find mavros)/launch/px4_pluginlists.yaml" />
    <arg name="config_yaml" default="$(find mavros)/launch/px4_config.yaml" />
    <group ns="uav'$i'">
        <arg name="fcu_url" default="udp://:'$mavoport'@'$ipaddress':'$mavport2'"/>
        <arg name="gcs_url" value=""/>
        <arg name="tgt_system" value="'$i'"/> 
        <arg name="tgt_component" value="1"/>
        <arg name="rcS1" default="$(find px4)/simulation_'$date_u'/$(arg vehicle)_'$i'"/>
        <arg name="ID" value="'$i'"/>
        <include file="$(find px4)/simulation_'$date_u'/single_vehicle.launch">
            <arg name="x" value="'$i'"/>
            <arg name="y" value="0"/>
            <arg name="z" value="0"/>
            <arg name="R" value="0"/>
            <arg name="P" value="0"/>
            <arg name="Y" value="0"/>
            <arg name="vehicle" value="$(arg vehicle)"/>
            <arg name="rcS" value="$(arg rcS1)"/>
            <arg name="mavlink_udp_port" value="'$PORT'"/>
            <arg name="ID" value="$(arg ID)"/>
        </include>
        <include file="$(find mavros)/launch/node.launch">
            <arg name="pluginlists_yaml" value="$(arg pluginlists_yaml)" />
            <arg name="config_yaml" value="$(arg config_yaml)" />

            <arg name="fcu_url" value="$(arg fcu_url)" />
            <arg name="gcs_url" value="$(arg gcs_url)" />
            <arg name="tgt_system" value="$(arg tgt_system)" />
            <arg name="tgt_component" value="$(arg tgt_component)" />
        </include>
    </group>
</launch>'>>simulation_$date_u/uav_mavros_sitl_$i.launch
	PORT=$((PORT+10))
done

# creation fichier launch pour monde gazebo vide :
echo '<launch>
    <arg name="debug" default="false"/>
    <arg name="verbose" default="false"/>
    <arg name="paused" default="false"/>
    <arg name="world" default="$(find mavlink_sitl_gazebo)/worlds/empty.world"/>
    <arg name="headless" default="false"/>
    <arg name="gui" default="true"/>
    <include file="$(find gazebo_ros)/launch/empty_world.launch">
        <arg name="headless" value="$(arg headless)"/>
        <arg name="gui" value="$(arg gui)"/>
        <arg name="world_name" value="$(arg world)" />
        <arg name="debug" value="$(arg debug)" />
        <arg name="verbose" value="$(arg verbose)" />
        <arg name="paused" value="$(arg paused)" />
    </include>
</launch>'>>simulation_$date_u/sitl_empty_gazebo_world_test.launch

# creation launch file vehicle_spawn :
echo '<launch>
    <arg name="x" default="0"/>
    <arg name="y" default="0"/>
    <arg name="z" default="0"/>
    <arg name="R" default="0"/>
    <arg name="P" default="0"/>
    <arg name="Y" default="0"/>
    <arg name="est" default="lpe"/>
    <arg name="vehicle" default="iris"/>
    <arg name="ID" default="1"/>
    <arg name="rcS" default="$(find px4)/simulation_'$date_u'/$(arg vehicle)_$(arg ID)"/>
    <arg name="mavlink_udp_port" default="'$PORT'" />
    <arg name="cmd" default="gz sdf -p simulation_'$date_u'/$(arg vehicle)_$(arg ID).urdf" /> 
    <param command="$(arg cmd)" name="$(arg vehicle)_$(arg ID)_sdf" />
    <node name="sitl_$(arg ID)" pkg="px4" type="px4" output="screen"
        args="$(find px4) $(arg rcS)">
    </node>
    <node name="$(arg vehicle)_$(arg ID)_spawn" output="screen" pkg="gazebo_ros" type="spawn_model"
        args="-sdf -param $(arg vehicle)_$(arg ID)_sdf -model $(arg vehicle)_$(arg ID) -x $(arg x) -y $(arg y) -z $(arg z) -R $(arg R) -P $(arg P) -Y $(arg Y)" respawn="false"/>
</launch>'>>simulation_$date_u/single_vehicle.launch


if [[ -n $norun ]]; then
	echo "script to launch simulation in folder simulation_"$date_u"/ :"
	echo "source Tools/setup_gazebo.bash $(pwd) $(pwd)/build_posix_sitl_default">>simulation_$date_u/step.sh
	echo "export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo">>simulation_$date_u/step.sh
	echo "export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)">>simulation_$date_u/step.sh
	echo "roslaunch simulation_$date_u/sitl_empty_gazebo_world_test.launch">>simulation_$date_u/step.sh
	for ((i=ID; i<=$((NUMBER+ID-1)); i++))
	do
		echo "roslaunch simulation_$date_u/uav_mavros_sitl_$i.launch">>simulation_$date_u/step.sh
	done
	exit 1
fi

#Sourcing environment
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build_posix_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)

#executing the launch files
if [[ -z $dist ]]; then #If the ip address parameter has been given, the empty gazebo world is not launched.
	gnome-terminal -e "roslaunch simulation_$date_u/sitl_empty_gazebo_world_test.launch"
	sleep 5
fi

#export ROS_IP=$(ip route get 8.8.8.8 | awk '{print $NF; exit}')
export ROS_IP=`hostname -I`
export ROS_HOSTNAME=`hostname -I`
for ((i=ID; i<=$((NUMBER+ID-1)); i++))
do
	export ROS_MASTER_URI="http://"$ipaddress":"11311""
	gnome-terminal -e "roslaunch simulation_$date_u/uav_mavros_sitl_$i.launch"
done
