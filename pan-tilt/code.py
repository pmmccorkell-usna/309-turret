# Patrick McCorkell
# April 2022
# US Naval Academy
# Robotics and Control TSD
#

print("Hello %s" %__name__)
from time import sleep
import atexit
from ew309 import EW309

pantilt = EW309()

# def pwm_controller():
# 	pantilt.resume_pwm()
# 	pantilt.start_controller(pantilt.convert_pwm,0.005)

# 	# pantilt.start_controller is blocking until the ticker loop is paused.
# 	#	Code rejoins here:
# 	pantilt.pause_pwm()


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
		print('test delay %0.2f, drive %0.2f' %(pausetime,effort))
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

	while(1):
		if (pantilt.manual_mode.value):
			pantilt.poll_inputs()
			pantilt.set_outputs_manual()
			sleep(0.01)
		else:
			print('auto mode starting')
			new_x,new_y,new_x_damp,new_y_damp = 0,0,0,0
			pantilt.set_outputs(new_x,new_y,new_x_damp,new_y_damp)
			# pwm_controller()

	exit_program('__main__')



