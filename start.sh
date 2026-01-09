sudo ip link set can0 down
# configure CAN 2.0 with 1mbps
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up


sudo ip link set can1 down
# configure CAN 2.0 with 1mbps
sudo ip link set can1 type can bitrate 1000000
sudo ip link set can1 up

./script/launch_unilateral.sh right_arm can0 can1
