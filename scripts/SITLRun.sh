# The text start with '#' is annotation, the following options are corresponding to Options on CopterSim

bash ./udp_port_free.sh

# Set the path of the RflySim tools
SHPath=$(cd $(dirname $0); pwd)
USRPath=${SHPath#/home/}
USRPath=${USRPath%%/*}
PSP_PATH=/home/${USRPath}/PX4PSP
PSP_PATH_LINUX=${PSP_PATH}

# Start index of vehicle number (should larger than 0)
# This option is useful for simulation with multi-computers
START_INDEX=1

# Total vehicle Number to auto arrange position
# TOTAL_COPTER=8 # Uncomment if needed

# Set the vehicleType/ClassID of vehicle 3D display in RflySim3D
CLASS_3D_ID=310

# Set use DLL model name or not, use number index or name string
# This option is useful for simulation with other types of vehicles instead of multicopters
DLLModel=0

# Check if DLLModel is a name string, if yes, copy the DLL file to CopterSim folder
if [ "$DLLModel" -gt -1 ] 2>/dev/null ;then 
	echo Use CopterSim internal model
else
      # "$1 is number." 
      echo Copy the latest dll/so file to CopterSim folder
      \cp -rf ${SHPath}/${DLLModel}.so ${PSP_PATH}/CopterSim/external/model/${DLLModel}.so
fi 

# Set the simulation mode on CopterSim, use number index or name string
# e.g., SimMode=2 equals to SimMode=PX4_SITL_RFLY
SimMode=2

# Set the vehicle-model (airframe) of PX4 SITL simulation, the default airframe is a quadcopter: iris
# Check folder Firmware/ROMFS/px4fmu_common/init.d-posix for supported airframes
PX4SitlFrame=iris

# Set the map, use index or name of the map on CopterSim
# e.g., UE4_MAP=1 equals to UE4_MAP=Grasslands
UE4_MAP=ChallengeMap

# Set the origin x,y position (m) and yaw angle (degree) at the map
ORIGIN_POS_X=-0.1
ORIGIN_POS_Y=-0.8
ORIGIN_YAW=0

# Set the interval between two vehicle, unit:m 
VEHICLE_INTERVAL=2

# Set broadcast to other computer; IS_BROADCAST=0: only this computer, IS_BROADCAST=1: broadcast; 
# or use IP address to increase speed, e.g., IS_BROADCAST=192.168.3.1
# Note: in IP mode, IS_BROADCAST=0 equals to IS_BROADCAST=127.0.0.1, IS_BROADCAST=1 equals to IS_BROADCAST=255.255.255.255
IS_BROADCAST=0

# Set UDP data mode; 0: UDP_FULL, 1:UDP_Simple, 2: Mavlink_Full, 3: Mavlink_simple. input number or string
# 4:Mavlink_NoSend, 5:Mavlink_NoGPS, 6:Mavlink_Vision (NoGPS and set PX4 EKF)
UDPSIMMODE=2

# Max vehicle number 50
MAX_VEHICLE=50

# Input vehicle number
VehicleNum=1
while true; do
    #read -p "Please input UAV swarm number: " VehicleNum
    Evaluated=$((VehicleNum))
    if [ "$Evaluated" -eq "$VehicleNum" ] && [ "$VehicleNum" -gt 0 ]; then
        if [ "$VehicleNum" -gt "$MAX_VEHICLE" ]; then
            echo "The vehicle number is too large, which may cause a crash"
            read -p "Press Enter to continue..."
        else
            break
        fi
    else
        echo "Not a positive integer"
    fi
done

VehicleTotalNum=$((VehicleNum + START_INDEX - 1))
if [ -z "${TOTOAL_COPTER+x}" ]; then
    TOTOAL_COPTER=$VehicleTotalNum
fi

sqrtNum=1
squareNum=1
while [ "$squareNum" -le "$TOTOAL_COPTER" ]; do
    squareNum=$((sqrtNum * sqrtNum))
    if [ "$squareNum" -gt "$TOTOAL_COPTER" ]; then
        break
    fi
    sqrtNum=$((sqrtNum + 1))
done

ld_path=$LD_LIBRARY_PATH
pkill -x QGroundControl || true
# 正式运行默认不启动 QGC；调试时设置 ENABLE_QGC=1。
if [ "${ENABLE_QGC:-0}" = "1" ]; then
    echo Start QGroundControl
    export LD_LIBRARY_PATH=/opt/gstreamer/lib/x86_64-linux-gnu:$PSP_PATH/QGroundControl/squashfs-root/Qt/libs:$ld_path
    cd "${PSP_PATH}/QGroundControl/squashfs-root" || exit 1
    ${CMDPre} ./AppRun >/dev/null 2>&1 &
    sleep 1
else
    echo Skip QGroundControl
fi

pkill -x RflySim3D || true
echo Start RflySim3D
cd ${PSP_PATH}/RflySimUE5/RflySim3D/Binaries/Linux
${CMDPre} ./RflySim3D >/dev/null 2>&1 &
sleep 5


pkill -x CopterSim || true
echo Kill all CopterSims

# Kill all running CopterSims
pkill -f "CopterSim"
echo "Kill all CopterSims"

# Start CopterSim for each vehicle
export LD_LIBRARY_PATH=$PSP_PATH/CopterSim/lib:$ld_path
cd "$PSP_PATH/CopterSim"
cntr=$START_INDEX
endNum=$((VehicleTotalNum + 1))

#while [ "$cntr" -lt "$endNum" ]; do
#    PosXX=$((((${cntr}-1) / ${sqrtNum})*${VEHICLE_INTERVAL} + ${ORIGIN_POS_X}))    
#    PosYY=$((((${cntr}-1) % ${sqrtNum})*${VEHICLE_INTERVAL} + ${ORIGIN_POS_Y}))
#    ./CopterSim 1 $cntr $CLASS_3D_ID $DLLModel $SimMode "$UE4_MAP" "$IS_BROADCAST" $PosXX $PosYY $ORIGIN_YAW 1 "$UDPSIMMODE" &
#    sleep 1
#    cntr=$((cntr + 1))
#done

scale=2
while [ "$cntr" -lt "$endNum" ]; do
    echo $UDPSIMMODE
    ./CopterSim 2 $cntr $CLASS_3D_ID $DLLModel $SimMode "$UE4_MAP" "$IS_BROADCAST" $ORIGIN_POS_X $ORIGIN_POS_Y $ORIGIN_YAW 1 "$UDPSIMMODE" &
    sleep 1
    cntr=$((cntr + 1))
done



# Set ToolChainType 1:Win10WSL 3:Cygwin

# if IS_BROADCAST not equals to 0, then set it to 1
if  [ "$IS_BROADCAST" -gt 0 ] 2>/dev/null ;then 
    if [$IS_BROADCAST -eq 0] 2>/dev/null ;then
        IS_BROADCAST=0
    else
        IS_BROADCAST=1
    fi
fi

echo Starting PX4 Build
cd ${PSP_PATH_LINUX}/Firmware
chmod +x BkFile/EnvOri.sh
chmod +x Tools/sitl_multiple_run_rfly.sh
./BkFile/EnvOri.sh
make px4_sitl_default
echo ${VehicleNum} ${START_INDEX} ${PX4SitlFrame}
./Tools/sitl_multiple_run_rfly.sh ${VehicleNum} ${START_INDEX} ${PX4SitlFrame}
read -p "Press any key to exit" aa



pkill -x px4 || true
pkill -x CopterSim || true
pkill -x QGroundControl || true
pkill -x RflySim3D || true

echo Start End.
