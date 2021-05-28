# ECED 3901 Robotics Project 
**Authors:** Usman Kamran, Ryan Brownlee, Omar Abdel Hameed  
**Team #:** 03  
**Course:** ECED 3901  
**University:** Dalhousie University  
**Date:** Thursday, May 27 2021  
**Purpose:** The goal of this project is to create a robot that can fight fires for our client.   

## To run this program on the robot, perform the following steps:

**In Terminal 1:**
1. roscore
2. ssh ubuntu@10.0.39.39
3. roslaunch diff_tf eced3901bot.launch

**In Terminal 2:**
1. roscd eced3901_2021_team03
2. python src/eced3901_dt1.py odom

## To run this program using the Gazebo emulator, perform the following steps:
**In Terminal 1:**
1. roscore
2. ssh ubuntu@10.0.39.39
3. roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch

**In Terminal 2:**
1. roscd eced3901_2021_team03
2. python src/eced3901_dt1.py odom