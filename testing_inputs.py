#!/usr/bin/env python
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
from numpy import array
import moveit_commander
import rosservice
import geometry_msgs.msg
from denso_cobotta_gripper.msg import GripperMoveAction, GripperMoveGoal
from denso_cobotta_driver.srv import GetMotorState

# NOTE: Before start this program, please launch denso_cobotta_bring.launch




#
# Parallel gripper
#
#gripper_parallel_open = 0.015
#gripper_parallel_close = 0.0
#gripper_parallel_speed = 10.0
#gripper_parallel_effort = 10.0

def joint_inputs():
	num_of_positions = int(input("Enter the number of positions needed: "))
	keycard = 0
	joints = []
	while num_of_positions > keycard
		joint1 = float(input("Enter the value of joint 1: "))
		j1 = joint1 /180 * math.pi
		joint2 = float(input("Enter the value of joint 2: "))
		j2 = joint2 /180 * math.pi
		joint3 = float(input("Enter the value of joint 3: "))
		j3 = joint3 /180 * math.pi
		joint4 = float(input("Enter the value of joint 4: "))
		j4 = joint4 /180 * math.pi
		joint5 = float(input("Enter the value of joint 5: "))
		j5 = joint5 /180 * math.pi
		joint6 = float(input("Enter the value of joint 6: "))
		j6 = joint6 /180 * math.pi
		pose = [j1, j2, j3, j4, j5, j6]
		joints.append(pose)
		num_of_positions = num_of_positions - 1
	list_conversion = array(joints)
	joint_goal = list_conversion
	print("The joints have moved, here are the values: ", joint_goal)
	arm_move(move_group, joint_goal)

def arm_move(move_group, joint_goal):
    pose_radian = [x / 180.0 * math.pi for x in joint_goal]
    move_group.go(pose_radian, wait=True)
    move_group.stop()


#def gripper_move(gripper_client, width, speed, effort):
#    goal = GripperMoveGoal()
#    goal.target_position = width
#    goal.speed = speed
#    goal.effort = effort
#    gripper_client.send_goal(goal)


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
#    gripper_client = actionlib.SimpleActionClient('/cobotta/gripper_move',
#                                                 GripperMoveAction)

	joint_inputs()


        if not is_simulation() and is_motor_running() is not True:
            print >> sys.stderr, "  Please motor on."
            continue

#        gripper_move(gripper_client, gripper_width,
#                    gripper_parallel_speed, gripper_parallel_effort)

    print("Bye...")
planning_frame = move_group.get_planning_frame()
print "=========== Planning frame: %s" % planning_frame
print "=========== The Joint Positions:", joints_packing_old
group_names = robot.get_group_names()
print "=========== Available Planning Groups:", group_names
print "=========== Current Pose:", move_group.get_current_pose().pose
pose_goal = move_group.get_current_pose().pose
pose_goal.position.x = 0.185
pose_goal.position.y = 0
move_group.go(pose_goal,wait=True)
move_group.stop()
#print joint_goal[1]
#print joint_goal[2]
#print joint_goal[3]
#print joint_goal[4]
#print joint_goal[5]
#print pose_goal[6]

