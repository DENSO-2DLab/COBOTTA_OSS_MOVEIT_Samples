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
from denso_cobotta_gripper.msg import GripperMoveAction, GripperMoveGoal
from denso_cobotta_driver.srv import GetMotorState, SetMotorState

#
# Poses
#
joints_packing_old = [30, 10, 54, 1, 118, -107]
joints_packing_new = [90, -30, 120, -170, -94, 0]
joints_home = [0, 30, 100, 0, 50, 0]

#
# Parallel gripper
#
gripper_parallel_open = 0.015
gripper_parallel_close = 0.0
gripper_parallel_speed = 10.0
gripper_parallel_effort = 10.0

def arm_move(move_group, joint_goal):
    pose_radian = [x / 180.0 * math.pi for x in joint_goal]
    move_group.go(pose_radian, wait=True)
    move_group.stop()


def gripper_move(gripper_client, width, speed, effort):
    goal = GripperMoveGoal()
    goal.target_position = width
    goal.speed = speed
    goal.effort = effort
    gripper_client.send_goal(goal)


def is_motor_running():
    rospy.wait_for_service('/cobotta/get_motor_state', 3.0)
    try:
        get_motor_state = rospy.ServiceProxy('/cobotta/get_motor_state', GetMotorState)
        res = get_motor_state()
        return res.state
    except rospy.ServiceException, e:
        print >> sys.stderr, "  Service call failed: %s" % e

def are_brakes_on():
	rospy.wait_for_service('/cobotta/get_brake_state', 3.0)
	try:
		get_brake_state = rospy.ServiceProxy('/cobotta/get_brake_state', GetBrakeState)
	res = get_brake_state()
	print(res)
	return res.state
	except: rospy.ServiceException, e:
	print >> sys.stderr, " Service call failed: %s" % math.e


def motor_status():
	rospy.wait_for_service('/cobotta/set_motor_state', 3.0)
	motorstate = int(input("Enter '1' for motor on or '0' for motor off: "))
	if motorstate == 1:
		set_motor_state = rospy.ServiceProxy('/cobotta/set_motor_state', SetMotorState)
		res = set_motor_state(True)
		print(res)
	elif motorstate == 0:
		set_motor_state = rospy.ServiceProxy('/cobotta/set_motor_state', SetMotorState)
		res = set_motor_state(False)
		print(res)
	else:
		print('Invalid input, please try again!')
		motor_status()


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
    gripper_client = actionlib.SimpleActionClient('/cobotta/gripper_move',
                                                  GripperMoveAction)


    print(os.path.basename(__file__) + " sets pose goal and moves COBOTTA.")
    motor_status()
    print("0: Old packing pose, 1: New packing pose, 2: Home pose, Others: E1xit")
    while True:
        input = raw_input("  Select the value: ")
        if input.isdigit():
            input = int(input)

        joints = []
        gripper_width = 0.0

        if input == 0:
            joints = joints_packing_old
            gripper_width = gripper_parallel_open
        elif input == 1:
            joints = joints_packing_new
            gripper_width = gripper_parallel_open
        elif input == 2:
            joints = joints_home
            gripper_width = gripper_parallel_close
        else:
            break

        if not is_simulation() and is_motor_running() is not True:
            print >> sys.stderr, "  Please motor on."
            continue

        gripper_move(gripper_client, gripper_width,
                     gripper_parallel_speed, gripper_parallel_effort)
        arm_move(move_group, joints)

    print("Bye...")


