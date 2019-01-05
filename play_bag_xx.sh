#! /bin/bash
sleep 1

rosbag play /home/xx/andyoyo/calib_ws/bag/my_data.bag /velodyne_points:=/points_raw

