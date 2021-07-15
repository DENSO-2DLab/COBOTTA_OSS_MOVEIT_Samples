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
from denso_cobotta_gripper.msg import VacuumMoveAction, VacuumMoveGoal
from denso_cobotta_driver.srv import GetMotorState
import time


#
# These are the poses the COBOTTA will be taking
#To change the poses, use Rviz to identify the pose and sub in the new joint values 
#
approach_grabing_pose = [-48, 71, 20, 40, 117, -34]    #Position for COBOTTA approaching the grabing of the object
grabing_pose = [-35, 78, 33, 34, 96, -38]    #Position for COBOTTA grabing the object
middle_pose = [0, 0, 90, 0, 90, 0]    #Position for COBOTTA to be "centered"
dance_pose_1 = [89, 32, 119, -169, 90, 105]    #Position for COBOTTA to move to and "show-off" its moves
dance_pose_2 = [85, 28, 71, -22, -90, -117]    #Position for COBOTTA to move to and "show-off" its moves
approach_placing_pose = [-2, 21, 44, 58, 105, 19]    #Position for COBOTTA approaching the placing of the object
placing_pose = [-5, 23, 64, 55, 97, 2]    #Position for COBOTTA placing the object

#
# These are the parameters the Vacuum gripper will abide by, both direction and power percentage
#The only manipulayion that can occurn with the Vacuum in the lines below is the poer percentage (40.0 to 100.0)
#

vacuum_blow = 1    #The vacuum will blow out air
vacuum_stop = 0    #The vacuum will stop blowing/sucking the air
vacuum_suction = -1    #The vacuum will suck up air
vacuum_power_percentage = 70.0    #The power_percentage will ensure how much power to performing when blowing/stoping/sucking of the air


#
#The function underneath [arm_move(move_group, joint_goal)] is what carries out the joint actions the COBOTTA will be performing
#

def arm_move(move_group, joint_goal):
    pose_radian = [x / 180.0 * math.pi for x in joint_goal]    #Converts the angles from the positions into radians
    move_group.set_max_acceleration_scaling_factor(1)    #Sets the servo motor's accleration to 100
    move_group.set_max_velocity_scaling_factor(0.85)    #Sets the servo motor's velocity to 85
    move_group.go(pose_radian, wait=True)    #Performs the action (moving to the position) of the COBOTTA's joints
    move_group.stop()    #Ensures the COBOTTA's joints will stop before moving on to the next action


#
#The function underneath [vacuum_move(vacuum_client, direction, power_percentage)] is what carries out the actions the vacuum will be performing
#

def vacuum_move(vacuum_client, direction, power_percentage):
    goal = VacuumMoveGoal()    #Sets the goal of the COBOTTA equal to the library's (denso_cobotta_gripper.msg) function [VacuumMoveGoal()] that will ensure the Vacuum will carry out actions
    goal.direction = direction   #Sets up the parameter (direction) to be stored in the goal
    goal.power_percentage = power_percentage    #Sets up the paramter (power_percentage) to be stored in the goal
    vacuum_client.send_goal(goal)    #Sends the goal to the vacuum_client (a variable that will send the paramters to the SimpleActionClient function to carry out the action)


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
    rospy.init_node("pick and place")    #The nodes are initialized
    moveit_commander.roscpp_initialize(sys.argv)    #the ROS moveit_commander is initialzed in C++
    robot = moveit_commander.RobotCommander()    #The RobotCommander function (in the moveit_commander library) is set equal to robot
    move_group = moveit_commander.MoveGroupCommander("arm")    #The MoveGroupCommander fucntion (in the moveit_commander library) identifies the 6 joints of the COBOTTA as Arm and is set equal to move_group
    vacuum_client = actionlib.SimpleActionClient('/cobotta/vacuum_move', VacuumMoveAction)    #The SimpleActionClient (in the actionlib library) identifies the vacuum and brings in the function [VacuumMoveAction()] to have the vacuum carry out actions, along with setting it all equal to the vacuum_client 


joints = []    #an empty array that will be used to store all the joint positions the COBOTTA will be using 
vacuum_direction = 0    #a variable set equal to zero, but will change  when changing the direction of the Vacuum


#
#This is portion of the program where the COBOTTA will carry out all the actions in an "infinite" loop. Press 'CTRL+C' once in the terminal to stop executing the program
#Note: After pressing 'CTRL+C', the COBOTTA will finish executing the loop and then stop. If the vacuum remains on, turn it off (use the Denso Wave Cobotta Documentation under the Rostopic commands section to turn off the vacuum)
#

while True:
	joints = middle_pose    #The empty joints array is stores the joint values of the middle_pose
	arm_move(move_group, joints)    #The arm_move(move_group, joints) function will move the joints position
	time.sleep(0.5)    #The COBOTTA will wait 0.5 seconds before moving on

	joints = approach_grabing_pose    #The empty joints array is stores the joint values of the appraoch_grabing_pose
	arm_move(move_group, joints)    #The arm_move(move_group, joints) function will move the joints position
	time.sleep(0.5)    #The COBOTTA will wait 0.5 seconds before moving on

	joints = grabing_pose    #The empty joints array is stores the joint values of the grabing_pose
	vacuum_direction = vacuum_suction    #the vacuum_direction has been changed to the vacuum_suction
	arm_move(move_group, joints)    #The arm_move(move_group, joints) function will move the joints position
	vacuum_move(vacuum_client, vacuum_direction, vacuum_power_percentage)    #the vacuum_move() function will have the vacuum carray out the action
	time.sleep(0.5)    #The COBOTTA will wait 0.5 seconds before moving on

	joints = approach_grabing_pose    #The empty joints array is stores the joint values of the appraoch_grabing_pose
	arm_move(move_group, joints)    #The arm_move(move_group, joints) function will move the joints position
	time.sleep(0.5)    #The COBOTTA will wait 0.5 seconds before moving on
	
	joints = middle_pose    #The empty joints array is stores the joint values of the middle_pose
	arm_move(move_group, joints)    #The arm_move(move_group, joints) function will move the joints position
	time.sleep(0.5)    #The COBOTTA will wait 0.5 seconds before moving on

	joints = dance_pose_2    #The empty joints array is stores the joint values of the dance_pose_2
	arm_move(move_group, joints)    #The arm_move(move_group, joints) function will move the joints position
	time.sleep(0.5)    #The COBOTTA will wait 0.5 seconds before moving on

	joints = dance_pose_1    #The empty joints array is stores the joint values of the dance_pose_1
	arm_move(move_group, joints)    #The arm_move(move_group, joints) function will move the joints position
	time.sleep(0.5)    #The COBOTTA will wait 0.5 seconds before moving on

	joints = dance_pose_2    #The empty joints array is stores the joint values of the dance_pose_2
	arm_move(move_group, joints)    #The arm_move(move_group, joints) function will move the joints position
	time.sleep(0.5)    #The COBOTTA will wait 0.5 seconds before moving on

	joints = approach_placing_pose    #The empty joints array is stores the joint values of the appraoch_placing_pose
	arm_move(move_group, joints)    #The arm_move(move_group, joints) function will move the joints position
	time.sleep(0.5)    #The COBOTTA will wait 0.5 seconds before moving on

	joints = placing_pose    #The empty joints array is stores the joint values of the placing_pose
	arm_move(move_group, joints)    #The arm_move(move_group, joints) function will move the joints position
	time.sleep(0.5)    #The COBOTTA will wait 0.5 seconds before moving on
	vacuum_direction = vacuum_stop    #the vacuum_direction has been changed to the vacuum_stop
	vacuum_move(vacuum_client, vacuum_direction, vacuum_power_percentage)    #the vacuum_move() function will have the vacuum carray out the action
	time.sleep(0.5)    #The COBOTTA will wait 0.5 seconds before moving on

	joints = approach_placing_pose    #The empty joints array is stores the joint values of the appraoch_placing_pose
	arm_move(move_group, joints)    #The arm_move(move_group, joints) function will move the joints position
	time.sleep(0.5)    #The COBOTTA will wait 0.5 seconds before moving on

	joints = middle_pose    #The empty joints array is stores the joint values of the middle_pose
	arm_move(move_group, joints)    #The arm_move(move_group, joints) function will move the joints position
	time.sleep(10)    #The COBOTTA will wait 10 seconds before moving on


	if not is_simulation() and is_motor_running() is not True:    #This if statement will check to see if both the is_simulation() and is_motor_running() functions are not True, if they are not True then a print statement will print "Please turn on motor." If the functions are true, then the program will continue
		print >> sys.stderr, "  Please turn on motor."
        continue

print("------------------------------------------------------------------------------------------------------------------------")
print("The End of The Program!")
