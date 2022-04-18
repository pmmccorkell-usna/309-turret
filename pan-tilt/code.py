# Patrick McCorkell
# April 2022
# US Naval Academy
# Robotics and Control TSD
#


print("Hello %s" %__name__)
from time import monotonic_ns, sleep
import board
from math import floor, ceil
import atexit
from digitalio import DigitalInOut, Direction
from json import loads,dumps
from ew309 import EW309
from serial import Read_Serial_JSON

pantilt = EW309()

json_data = {
# 	'target':0,
# 	'Kp':4000,
# 	'Ki':0,
# 	'Kd':0,
# 	'runtime':1,
# 	'min_drive':mot_min_drive
}

ser = Read_Serial_JSON(json_data)

def your_controller():
	# Initialize all the variables you can control:
	pan_dir = 0			# 0 clockwise, 1 ccw
	pan_speed = 0		# [0,255]
	pan_damp = 0		# [0,255]

	tilt_dir = 0		# 0 down, 1 up
	tilt_speed = 0		# [0,255]
	tilt_damp = 0		# [0,255]

	# Maybe pull some PID values from matlab over serial ?
	some_characteristics_for_your_controller = ser.read_serial()

	print(pantilt.euler())
	sleep(0.3)

	##############################################
	####################### THINGS HAPPEN HERE
	##############################################

	pantilt.set_outputs(pan_speed,tilt_speed,pan_damp,tilt_damp)


####################################
######### Shutdown Section #########
####################################


# # Deinit all the GPIO. Runs automatically at the end.
def exit_program(source):
	print("Quitting program per %s." %source)
	# test_func_1()
	# test_func_2()
	pantilt.deinit_all()

# If code fails or reaches end, execute exit_program() with argument 'atexit'.
atexit.register(exit_program,'atexit')



###################################
######## Startup Section ##########
###################################


# Under REPL, import code has name 'code'
if __name__ == 'code':
	print('REPL detected')

	# Function to reload code.py in REPL after saving changes.
	# User may then 'import code' again to have those changes be live.
	#	   - cannot 'import code' from within code.py
	#
	# Usage:
	#	   - import code
	#	   - make changes in IDE, ctrl+s
	#	   - code.reload() to delete the registry
	#	   - import code to bring in the new changes
	#
	def reload():
		exit_program('REPL')
		del sys.modules['code']
	print('reload available')

	# perform a little dance to test the system.
	def test(pausetime=0.3,effort=0.1):
		pantilt.set_outputs()
		sleep(pausetime)
		pantilt.set_outputs(-1 * effort,-1 * effort)
		sleep(pausetime)
		pantilt.set_outputs(-1 * effort,effort)
		sleep(pausetime)
		pantilt.set_outputs(effort,effort)
		sleep(pausetime)
		pantilt.set_outputs(effort,-1 * effort)
		sleep(pausetime)
		pantilt.set_outputs()
	test()

# This is the default that runs when Ras Pico is plugged in, restarted, ctrl+s, etc.
if __name__ == '__main__':
	print("enter data")
	print(">>")
	# from controller import *

	while(1):
		if (pantilt.manual_mode.value):
			ser.clear_serial()
			pantilt.poll_inputs()
			pantilt.set_outputs_manual()
			print(pantilt.imu.euler)
			sleep(0.01)
		else:
			print('auto mode entered')
			your_controller()
			sleep(0.01)

	exit_program('__main__')



