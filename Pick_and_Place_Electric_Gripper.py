#Pick and Place Simulation in Python
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
from denso_cobotta_driver.srv import GetMotorState
import time


#
# These are the poses the COBOTTA will be taking
#To change the poses, use Rviz to identify the pose and sub in the new joint values 
#
calset_pose = [150, -60, 140, -170, 135, 170]    #Position for COBOTTA to execute the CALSET
approach_grabing_pose = [-40, 61, 53, 30, 100, 40]    #Position for COBOTTA approaching the grabing of the object
grabing_pose = [-28, 66, 75, 38, 73, 32]    #Position for COBOTTA grabing the object
middle_pose = [0, 0, 90, 0, 90, 0]    #Position for COBOTTA to be "centered"
dance_pose_1 = [89, 32, 119, -169, 90, 105]    #Position for COBOTTA to move to and "show-off" its moves
dance_pose_2 = [85, 28, 71, -22, -90, -117]    #Position for COBOTTA to move to and "show-off" its moves
approach_placing_pose = [-5, 5, 66, 65, 97, 100]    #Position for COBOTTA approaching the placing of the object
placing_pose = [24, 7, 92, 40, 61, 101]    #Position for COBOTTA placing the object


#
# Parallel gripper
#
gripper_open = 0.015  #The Electric Gripper fully opens
gripper_close = 0.0    #The Electric Gripper fully closes
gripper_speed = 50.0    #The Gripper opens and closes at 50% speed
gripper_effort = 20.0    #The gripper uses 20 Newtons of Force


#
#The function underneath [arm_move(move_group, joint_goal)] is what carries out the joint actions the COBOTTA will be performing
#
def arm_move(move_group, joint_goal):
    pose_radian = [x / 180.0 * math.pi for x in joint_goal]    #Converts the angles from the positions into radians
    move_group.set_max_acceleration_scaling_factor(0.75)    #Sets the servo motor's accleration to 100
    move_group.set_max_velocity_scaling_factor(0.65)    #Sets the servo motor's velocity to 85
    move_group.go(pose_radian, wait=True)    #Performs the action (moving to the position) of the COBOTTA's joints
    move_group.stop()    #Ensures the COBOTTA's joints will stop before moving on to the next action


#
#The function underneath [gripper_move(gripper_client, width, speed, effort)] is what carries out the movement of the Electric Gripper
#
def gripper_move(gripper_client, width, speed, effort):
    goal = GripperMoveGoal()    #The GripperMoveGoal() function is set equal to the goal variable
    goal.target_position = width    #The target position of the goal is set equal to the width of the gripper (how much it opens and closes)
    goal.speed = speed    #The speed of the goal is set equal to the speed of how fats the gripper will move
    goal.effort = effort    #The effort of the goal is set equal to the effort of how much force the gripper will apply 
    gripper_client.send_goal(goal)    #The goal is then sent through the gripper_client function


#
#The function underneath [is_motor_running()] will check to see if the motor of the COBOTTA are on/off
#
def is_motor_running():
    rospy.wait_for_service('/cobotta/get_motor_state', 3.0)   #wait_for_service blocks the service call unitl available.
    try:
        get_motor_state = rospy.ServiceProxy('/cobotta/get_motor_state', GetMotorState)    #ServiceProxy creates a reference to a ROS service for supported calls
        res = get_motor_state()    #The obtained state of the motor is set equal to res
        return res.state    #The result of the motor state is returned
    except rospy.ServiceException, e:
        print >> sys.stderr, "  Service call failed: %s" % e    #If the service call fails to be avaliable to get the state of the motor, "Service call failed" will be printed along with an error number.


#
#The function underneath [is_simulation()] will check and see if the get_motor_state rosserice command is in the service list
#
def is_simulation():
    service_list = rosservice.get_service_list()    #The obtaining of the rosserivce list is set equal to service_list
    if '/cobotta/get_motor_state' in service_list:    #If the get_motor_state rosservice is within the service_list, return a False, if not return a True
        return False
    return True


#
#This is the main program where all the libraries and nodes are initilaized, functions are set equal to variables for easy programming, and certain groups/functions/commands are identified.
#

if __name__ == '__main__':
    rospy.init_node("Pick_and_Place_with_Electric_Gripper")    #The nodes are initialized
    moveit_commander.roscpp_initialize(sys.argv)    #the ROS moveit_commander is initialzed in C++
    robot = moveit_commander.RobotCommander()    #The RobotCommander function (in the moveit_commander library) is set equal to robot
    move_group = moveit_commander.MoveGroupCommander("arm")    #The MoveGroupCommander fucntion (in the moveit_commander library) identifies the 6 joints of the COBOTTA as Arm and is set equal to move_group
    gripper_client = actionlib.SimpleActionClient('/cobotta/gripper_move', GripperMoveAction)    #The SimpleActionClient (in the actionlib library) identifies the gripper and brings in the function [GripperMoveAction()] to have the gripper carry out actions, along with setting it all equal to the gripper_client 


joints = []    #an empty array that will be used to store all the joint positions the COBOTTA will be using 
gripper_width = 0.0    #a variable set equal to zero, but will change  when changing the direction of the Gripper

joints = calset_pose    #The empty joints array is stores the joint values of the calset_pose
arm_move(move_group, joints)    #The arm_move(move_group, joints) function will move the joints position
time.sleep(5)    #The COBOTTA will wait 5 seconds before moving on

#
#This is portion of the program where the COBOTTA will carry out all the actions in an "infinite" loop. Press 'CTRL+C' once in the terminal to stop executing the program
#Note: After pressing 'CTRL+C', the COBOTTA will finish executing the loop and then stop. If the gripper remains on, turn it off (use the Denso Wave Cobotta Documentation under the Rostopic commands section to turn off the gripper)
#

while True:
	joints = middle_pose    #The empty joints array is stores the joint values of the middle_pose
	arm_move(move_group, joints)    #The arm_move(move_group, joints) function will move the joints position
	time.sleep(3)    #The COBOTTA will wait 5 seconds before moving on

	joints = approach_grabing_pose    #The empty joints array is stores the joint values of the appraoch_grabing_pose
	arm_move(move_group, joints)    #The arm_move(move_group, joints) function will move the joints position
	time.sleep(3)    #The COBOTTA will wait 5 seconds before moving on

	joints = grabing_pose    #The empty joints array is stores the joint values of the grabing_pose
	gripper_width = gripper_close    #the gripper_width has been changed to the gripper_close
	arm_move(move_group, joints)    #The arm_move(move_group, joints) function will move the joints position
	gripper_move(gripper_client, gripper_width, gripper_speed, gripper_effort)    #the gripper_move() function will have the gripper carray out the action
	time.sleep(3)    #The COBOTTA will wait 5 seconds before moving on

	joints = approach_grabing_pose    #The empty joints array is stores the joint values of the appraoch_grabing_pose
	arm_move(move_group, joints)    #The arm_move(move_group, joints) function will move the joints position
	time.sleep(3)    #The COBOTTA will wait 5 seconds before moving on
	
	joints = middle_pose    #The empty joints array is stores the joint values of the middle_pose
	arm_move(move_group, joints)    #The arm_move(move_group, joints) function will move the joints position
	time.sleep(3)    #The COBOTTA will wait 5 seconds before moving on

	joints = dance_pose_2    #The empty joints array is stores the joint values of the dance_pose_2
	arm_move(move_group, joints)    #The arm_move(move_group, joints) function will move the joints position
	time.sleep(3)    #The COBOTTA will wait 5 seconds before moving on

	joints = dance_pose_1    #The empty joints array is stores the joint values of the dance_pose_1
	arm_move(move_group, joints)    #The arm_move(move_group, joints) function will move the joints position
	time.sleep(3)    #The COBOTTA will wait 5 seconds before moving on

	joints = dance_pose_2    #The empty joints array is stores the joint values of the dance_pose_2
	arm_move(move_group, joints)    #The arm_move(move_group, joints) function will move the joints position
	time.sleep(3)    #The COBOTTA will wait 5 seconds before moving on

	joints = approach_placing_pose    #The empty joints array is stores the joint values of the appraoch_placing_pose
	arm_move(move_group, joints)    #The arm_move(move_group, joints) function will move the joints position
	time.sleep(3)    #The COBOTTA will wait 5 seconds before moving on

	joints = placing_pose    #The empty joints array is stores the joint values of the placing_pose
	arm_move(move_group, joints)    #The arm_move(move_group, joints) function will move the joints position
	time.sleep(3)    #The COBOTTA will wait 3 seconds before moving on
	gripper_width = gripper_open    #the gripper_width has been changed to the gripper_open
	gripper_move(gripper_client, gripper_width, gripper_speed, gripper_effort)    #the gripper_move() function will have the gripper carray out the action
	time.sleep(3)    #The COBOTTA will wait 5 seconds before moving on

	joints = approach_placing_pose    #The empty joints array is stores the joint values of the appraoch_placing_pose
	arm_move(move_group, joints)    #The arm_move(move_group, joints) function will move the joints position
	time.sleep(3)    #The COBOTTA will wait 5 seconds before moving on

	joints = middle_pose    #The empty joints array is stores the joint values of the middle_pose
	arm_move(move_group, joints)    #The arm_move(move_group, joints) function will move the joints position
	time.sleep(30)    #The COBOTTA will wait 10 seconds before moving on


	if not is_simulation() and is_motor_running() is not True:    #This if statement will check to see if both the is_simulation() and is_motor_running() functions are not True, if they are not True then a print statement will print "Please turn on motor." If the functions are true, then the program will continue
		print >> sys.stderr, "  Please turn on motor."
        	continue

print("------------------------------------------------------------------------------------------------------------------------")
print("The End of The Program!")
