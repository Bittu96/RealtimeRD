#!/bin/bash

roslaunch humanoid_gazebo humanoid_world.launch

roslaunch humanoid_moveit moveit_planning_execution.launch
