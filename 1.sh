sudo modprobe gs_usb
sudo ip link set can0 up type can bitrate 500000

#roslaunch scout_bringup open_rslidar.launch



#roslaunch work_eins odometry_estimator
#roslaunch work_eins velocity_display
