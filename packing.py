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
approach_grabing_pose = [-48, 71, 20, 40, 117, -34]
grabing_pose = [-35, 78, 33, 34, 96, -38]
middle_pose = [0, 0, 90, 0, 90, 0]
dance_pose_1 = [89, 32, 119, -169, 90, 105]
dance_pose_2 = [85, 28, 71, -22, -90, -117]
approach_placing_pose = [-2, 21, 44, 58, 105, 19]
placing_pose = [-5, 23, 64, 55, 97, 2]

#
# Parallel gripper
#

vacuum_blow = 1
vacuum_stop = 0
vacuum_suction = -1
vacuum_power_percentage = 70.0

def arm_move(move_group, joint_goal):
    pose_radian = [x / 180.0 * math.pi for x in joint_goal]
    move_group.set_max_acceleration_scaling_factor(1)
    move_group.set_max_velocity_scaling_factor(0.85)
    move_group.go(pose_radian, wait=True)
    move_group.stop()


def vacuum_move(vacuum_client, direction, power_percentage):
    goal = VacuumMoveGoal()
    goal.direction = direction
    goal.power_percentage = power_percentage
    vacuum_client.send_goal(goal)

def insert_slide():
    slide_pose = geometry_msgs.msg.PoseStamped()
    slide_pose.header.frame_id = "world"
    slide_pose.pose.orientation.w = 0.5
    slide_pose.pose.position.z = 0.07
    slide_name = "boxslide.STL"
    scene.add_mesh(slide_name, slide_pose, boxslide.STL, size=(1, 1, 1))

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
    #scene = moveit_commander.PlanningSceneInterface()
    move_group = moveit_commander.MoveGroupCommander("arm")
    vacuum_client = actionlib.SimpleActionClient('/cobotta/vacuum_move', VacuumMoveAction)
    #print robot.get_current_state()

joints = []
vacuum_direction = 0
key = 3
while(key > 0):
	#insert_slide()
	joints = middle_pose
	arm_move(move_group, joints)
	time.sleep(0.5)

	joints = approach_grabing_pose
	arm_move(move_group, joints)
	time.sleep(0.5)

	joints = grabing_pose
	vacuum_direction = vacuum_suction
	arm_move(move_group, joints)
	vacuum_move(vacuum_client, vacuum_direction, vacuum_power_percentage)
	time.sleep(0.5)

	joints = approach_grabing_pose
	arm_move(move_group, joints)
	time.sleep(0.5)
	
	joints = middle_pose
	arm_move(move_group, joints)
	time.sleep(0.5)

	joints = dance_pose_2
	arm_move(move_group, joints)
	time.sleep(0.5)

	joints = dance_pose_1
	arm_move(move_group, joints)
	time.sleep(0.5)

	joints = dance_pose_2
	arm_move(move_group, joints)
	time.sleep(0.5)

	joints = approach_placing_pose
	arm_move(move_group, joints)
	time.sleep(0.5)

	joints = placing_pose
	arm_move(move_group, joints)
	time.sleep(0.5)
	vacuum_direction = vacuum_stop
	vacuum_move(vacuum_client, vacuum_direction, vacuum_power_percentage)
	time.sleep(0.5)

	joints = approach_placing_pose
	arm_move(move_group, joints)
	time.sleep(0.5)

	joints = middle_pose
	arm_move(move_group, joints)
	time.sleep(0.5)

	key = key - 1
	if not is_simulation() and is_motor_running() is not True:
		print >> sys.stderr, "  Please motor on."
        continue
	#vacuum_move(vacuum_client, vacuum_direction, vacuum_power_percentage)
	#arm_move(move_group, joints)
print("The End!!!!!!!!!!!!!!!")
