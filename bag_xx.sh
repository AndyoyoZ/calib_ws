#! /bin/bash
sleep 1
rosbag record -O /home/andyoyo/calib_ws/bag/my_data  /velodyne_points /scan /camera/image

#rosbag play /home/andyoyo/calib_ws/bag/my_data.bag   -r 0.5

