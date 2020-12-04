# Packman hardware driver
This driver emulates a bluebotic BlueBotics ANT lite+ so that it can control the wheels of the packman robot.

## Working with the the USB-to-CAN v2
__disable secure boot__. The Driver below is not signed, so you will not be able to load it when secure boot is enabled.

Install this kernel driver: https://github.com/jackthompsonnz/socketcan-linux-5.0

Check if you can see the device
```
ip link
```
Then configure the device and start it (this should be automated later with a netplan config)
```
sudo ip link set can0 type can bitrate 250000
sudo ip link set can0 up
```

## Working with a virtual can interface
https://blog.mbedded.ninja/programming/operating-systems/linux/how-to-use-socketcan-with-the-command-line-in-linux/
```sh
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0
watch cansend vcan0 182#1122334455667788
```

## ROS2 loading controllers
```sh
ros2 control load joint_state_controller
ros2 control load forward_command_controller
ros2 control switch --start-controllers joint_state_controller forward_command_controller
```
Sending a command
```sh
ros2 topic pub /forward_command_controller/commands std_msgs/msg/Float64MultiArray "data:
- 0.5
- 0.5"
```
