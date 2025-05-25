#!/bin/bash

# 删除nav2进程

# nav2_container进程
PID_nav=$(ps -eo pid,lstart,etime,cmd | grep nav2 | grep -v grep | awk '{ print $1 }')
# gzclient进程
PID_gaz=$(ps -eo pid,lstart,etime,cmd | grep gz | grep -v grep | awk '{ print $1 }')

if [ -z "${PID_nav}" ]
then
        echo Nav2 is already stopped
else
        echo "kill Nav2 ${PID_nav}"
        kill -9 ${PID_nav}
fi

if [ -z "${PID_gaz}" ]
then
        echo Gazebo is already stopped
else
        echo "kill Gazebo ${PID_gaz}"
        kill -9 ${PID_gaz}
fi
