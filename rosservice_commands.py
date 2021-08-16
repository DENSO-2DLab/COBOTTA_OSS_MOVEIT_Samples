# usage: python ./rosservice_commands.py
#
#!/usr/bin/env python
import os
import sys
import rospy
import rosservice
from denso_cobotta_driver.srv import GetMotorState, SetMotorState, GetBrakeState, SetBrakeState, SetLEDState, ClearError, ClearRobotError, ClearSafeState

#
#The following functions are based on the rosservice commands
#

#Checks the status of the Motor Power (This function was orignially named is_motor_running by DENSO-Wave)
def motor_power_status():
    rospy.wait_for_service('/cobotta/get_motor_state', 3.0)
    try:
        print('This is the Motor Power State:')
        get_motor_state = rospy.ServiceProxy('/cobotta/get_motor_state', GetMotorState)
        res = get_motor_state()
	print(res)
        return res.state
    except rospy.ServiceException, e:
        print >> sys.stderr, "  Service call failed: %s" % e

#sets motor power to on or off
def motor_power_switch():
	rospy.wait_for_service('/cobotta/set_motor_state', 3.0)
	motorstate = int(input("Enter '1' for motor on or '0' for motor off: "))
	if motorstate == 1:
		set_motor_state = rospy.ServiceProxy('/cobotta/set_motor_state', SetMotorState)
		res = set_motor_state(True)
		print(res)
		print('Motor is ON!')
	elif motorstate == 0:
		set_motor_state = rospy.ServiceProxy('/cobotta/set_motor_state', SetMotorState)
		res = set_motor_state(False)
		print(res)
		print('Motor is OFF!')
	else:
		print('Invalid input, please try again!')
		motor_status()

#checks the status of the brakes
def brakes_status():
	rospy.wait_for_service('/cobotta/get_brake_state', 3.0)
	try:
                print('This is the Brakes State:')
		get_brake_state = rospy.ServiceProxy('/cobotta/get_brake_state', GetBrakeState)
		res = get_brake_state()
		print(res)
		return res.state
	except rospy.ServiceException, e:
		print >> sys.stderr, " Service call failed: %s" % e

#sets brakes to on or off
def brakes_switch():	
	rospy.wait_for_service('/cobotta/set_brake_state', 3.0)
	brakestate = int(input("Enter '1' for brakes to be on or '0' for brakes to be off: "))
	if brakestate == 1:
		set_brake_state = rospy.ServiceProxy('/cobotta/set_brake_state', SetBrakeState)
		res = set_brake_state([True, True, True, True, True, True])
		print(res)
		print('Brakes are ON!')
	elif brakestate == 0:
		set_brake_state = rospy.ServiceProxy('/cobotta/set_brake_state', SetBrakeState)
		res = set_brake_state([False, False, False, False, False, False])
		print(res)
		print('Brakes are OFF!')
	else:
		print('Invalid input,please try again!')
		brake_status()

#Maniplautes the LED's colors and blinking rate
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

#
#The main program (where it all runs)
#
if __name__ == '__main__':
    rospy.init_node("rosservice_commands")

    print(os.path.basename(__file__) + " uses the rosservice commands from the terminal in python.")
    motor_power_status()
    motor_power_switch()
    brakes_status()
    brakes_switch()
    manipulate_LED()
    error_clearer()

    print("The END of the rosservice commands in python!")
    print('----------------------------------------------------------------------------------------------------------------------------------------------------------------')
