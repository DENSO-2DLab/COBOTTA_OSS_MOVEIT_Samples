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
import rostopic
import geometry_msgs.msg
from denso_cobotta_gripper.msg import GripperMoveAction, GripperMoveGoal
from denso_cobotta_driver.srv import GetMotorState, SetMotorState, GetBrakeState, SetBrakeState, ExecCalset, SetLEDState, ClearError, ClearRobotError, ClearSafeState

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

#checks if brakes are on/off
def are_brakes_on():
	rospy.wait_for_service('/cobotta/get_brake_state', 3.0)
	try:
		get_brake_state = rospy.ServiceProxy('/cobotta/get_brake_state', GetBrakeState)
		res = get_brake_state()
		print(res)
		return res.state
	except rospy.ServiceException, e:
		print >> sys.stderr, " Service call failed: %s" % e

#sets brakes on/off
def brakes_status():	
	rospy.wait_for_service('/cobotta/set_brake_state', 3.0)
	brakestate = int(input("Enter '1' for brakes to be on or '0' for brakes to be off: "))
	if brakestate == 1:
		set_brake_state = rospy.ServiceProxy('/cobotta/set_brake_state', SetBrakeState)
		res = set_brake_state([True, True, True, True, True, True])
		print(res)
	elif brakestate == 0:
		set_brake_state = rospy.ServiceProxy('/cobotta/set_brake_state', SetBrakeState)
		res = set_brake_state([False, False, False, False, False, False])
		print(res)
	else:
		print('Invalid input,please try again!')
		brake_status()

#clear errors
def error_clearer():
	rospy.wait_for_service('/cobotta/clear_error', 3.0)
	rospy.wait_for_service('/cobotta/clear_robot_error', 3.0)
	rospy.wait_for_service('/cobotta/clear_safe_state', 3.0)
	check = int(input("Would you like to clear an error?\nEnter '1' for Yes or '0' for No: "))
	if check == 0:
		print('No errors here, moving on!')
	elif check == 1:
		error = int(input("What clearing error option do you want to use?\nEnter '1' for clear error, '2' for clear robot error, '3' for clear safe state: "))
		if error == 1:
			clear_error = rospy.ServiceProxy('/cobotta/clear_error', ClearError)
			res = clear_error()
			print(res)
		elif error == 2:
			clear_robot_error = rospy.ServiceProxy('/cobotta/clear_robot_error', ClearRobotError)
			res = clear_robot_error()
			print(res)
		elif error == 3:
			clear_safe_state = rospy.ServiceProxy('/cobotta/clear_safe_error', ClearSafeState)
			res = clear_safe_state()
			print(res)
		else:
			print("Invalid input! Try Again!")
			error_clearer()
	else:
		print("Invalid input! Try again!")
		error_clearer()

#sets motor on/off
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

#execute a calset
def calset_execution():
	exec_calset = rospy.ServiceProxy('/cobotta/exec_calset', ExecCalset)
	res = exec_calset()
	print(res)

#Maniplautes the LEDs
def manipulate_LED():
	rospy.wait_for_service('/cobotta/set_LED_state', 3.0)
	reds = int(input("Enter the amount of Red to use from 0 - 255: "))
	if reds > 255:
		print("Invalid Number! I said 0 - 255, START OVER!")
		manipulate_LED()
	elif reds < 0:
		print("Invalid Number! I said 0 - 255, START OVER!")
		manipulate_LED()
	else:
		print('Good Choice!')
	greens = int(input("Enter the amount of Green to use from 0 - 255: "))
	if greens > 255:
		print("Invalid Number! I said 0 - 255, START OVER!")
		manipulate_LED()
	elif greens < 0:
		print("Invalid Number! I said 0 - 255, START OVER!")
		manipulate_LED()
	else:
		print('Good Choice!')
	blues = int(input("Enter the amount of Blue to use from 0 - 255: "))
	if blues > 255:
		print("Invalid Number! I said 0 - 255, START OVER!")
		manipulate_LED()
	elif blues < 0:
		print("Invalid Number! I said 0 - 255, START OVER!")
		manipulate_LED()
	else:
		print('Good Choice!')
	blink_rates = int(input("Enter the amount of how fast to blink from 0 - 255: "))
	if blink_rates > 255:
		print("Invalid Number! I said 0 - 255, START OVER!")
		manipulate_LED()
	elif blink_rates < 0:
		print("Invalid Number! I said 0 - 255, START OVER!")
		manipulate_LED()
	else:
		print('Good Choice!')
	set_LED_state = rospy.ServiceProxy('/cobotta/set_LED_state', SetLEDState)
	res = set_LED_state(reds, greens, blues, blink_rates)
	print(res)

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
    are_brakes_on()
    brakes_status()
    error_clearer()
    manipulate_LED()
    calset_execution()
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


