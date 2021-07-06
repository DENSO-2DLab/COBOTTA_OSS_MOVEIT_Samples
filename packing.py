#
# Copyright (C) 2019  DENSO WAVE INCORPORATED
#
# -*- coding: utf-8 -*-
#
# usage: python ./packing_pose.py
#
#!/usr/bin/env python
import os
import sys
import rospy
import actionlib
import math
import moveit_commander
import rosservice
import geometry_msgs.msg
from denso_cobotta_gripper.msg import VacuumMoveAction, VacuumMoveGoal
from denso_cobotta_driver.srv import GetMotorState
import time

# NOTE: Before start this program, please launch denso_cobotta_bring.launch

#joints_name = ["joint_1", "joint_2",
#               "joint_3", "joint_4", "joint_5", "joint_6"]

#
# Poses
#
grabing_pose = [-52, 85, 22, -107, -82, 162]
middle_pose = [-57, -28, 103, -7, 107, 162]
placing_pose = [-52, 27, 81, -26, 106, 162]

#
# Parallel gripper
#
vacuum_blow = 1
vacuum_stop = 0
vacuum_suction = -1
vacuum_power_percentage = 100.0

def arm_move(move_group, joint_goal):
    pose_radian = [x / 180.0 * math.pi for x in joint_goal]
    move_group.go(pose_radian, wait=True)
    move_group.stop()


def vacuum_move(vacuum_client, direction, power_percentage):
    goal = VacuumMoveGoal()
    goal.direction = direction
    goal.power_percentage = power_percentage
    vacuum_client.send_goal(goal)


def is_motor_running():
    rospy.wait_for_service('/cobotta/get_motor_state', 3.0)
    try:
        get_motor_state = rospy.ServiceProxy('/cobotta/get_motor_state',
                                             GetMotorState)
        res = get_motor_state()
        return res.state
    except rospy.ServiceException, e:
        print >> sys.stderr, "  Service call failed: %s" % e

def is_simulation():
    service_list = rosservice.get_service_list()
    if '/cobotta/get_motor_state' in service_list:
        return False
    return True

if __name__ == '__main__':
    rospy.init_node("packing_pose")
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    move_group = moveit_commander.MoveGroupCommander("arm")
    vacuum_client = actionlib.SimpleActionClient('/cobotta/vacuum_move',
                                                  VacuumMoveAction)

joints = []
vacuum_direction = 0
key = 3
while(key >= 0):
	joints = grabing_pose
	vacuum_dircetion = vacuum_suction
	time.sleep(3)
	
	joints = middle_pose
	time.sleep(3)

	joints = placing_pose
	time.sleep(1)
	vacuum_direction = vacuum_stop
	time.sleep(3)

	joints = middle_pose
	time.sleep(1)

	key = key - 1

        if not is_simulation() and is_motor_running() is not True:
            print >> sys.stderr, "  Please motor on."
            continue

        vacuum_move(vacuum_client, vacuum_direction, vacuum_power_percentage)
        arm_move(move_group, joints)
        print("The End!!!!!!!!!!!!!!!")
