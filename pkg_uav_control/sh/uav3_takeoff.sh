#!/bin/bash

rostopic pub -1 /uav3/cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 1.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}'

sleep 3

rostopic pub -1 /uav3/cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}'

