# Patrick McCorkell
# April 2022
# US Naval Academy
# Robotics and Control TSD
#


print("Hello %s" %__name__)
from time import monotonic_ns, sleep
import board
import busio
import analogio
import adafruit_ds3502
from math import floor, ceil
import atexit
from digitalio import DigitalInOut, Direction
from json import loads,dumps

###################################
############ SPI Setup ############
###################################

# Chip Select for 1st MAX-522 chip.
cs_1 = DigitalInOut(board.IO1)
cs_1.direction = Direction.OUTPUT
cs_1.value = 1

# Chip Select for 2nd MAX-522 chip.
cs_2 = DigitalInOut(board.IO3)
cs_2.direction = Direction.OUTPUT
cs_2.value = 1

# Setup SCK and MOSI on default pins.
spi = busio.SPI(board.SCK, MOSI=board.MOSI)

# Acquire the SPI bus.
while not spi.try_lock():
	pass

# Per Max522 DAC datasheet page 10: ph0, pol0, freq up to 500kHz.
spi.configure(baudrate=500000, phase=0, polarity=0)

# Write 2 8bit values over the SPI bus.
# Per Max522 DAC datasheet page 8.
def spi_write(val1,val2):
	# Ensure the data is clamped to 8 bits each before sending.
	val1 = int(clamp_val(val1,0,255))
	val2 = int(clamp_val(val2,0,255))
	# print('spi_write:')
	# print(val1,val2)
	# Send it.
	spi.write(bytes([val1,val2]))
	sleep(0.01)


###################################
########## MAX-522 Setup ##########
###################################

# 1st MAX522 DAC chip.
def max522_dac1_write(chan,data):
	cs_1.value = 0
	# print('dac1')
	spi_write(chan,data)
	cs_1.value = 1

# Ch A on 1st MAX522 DAC chip.
def max522_dac1_chA(data):
	# if (data<25):
	# 	max522_dac1_write(8,0)
	# else:
	max522_dac1_write(1,data)

# Ch B on 1st MAX522 DAC chip.
def max522_dac1_chB(data):
	# if (data<25):
	# 	max522_dac1_write(16,0)
	# else:
	max522_dac1_write(2,data)

# 2nd MAX522 DAC chip
def max522_dac2_write(chan,data):
	cs_2.value = 0
	# print('dac2')
	spi_write(chan,data)
	cs_2.value = 1

# Ch A on 2nd MAX522 DAC chip.
def max522_dac2_chA(data):
	# if (data<25):
	# 	max522_dac2_write(8,0)
	# else:
	max522_dac2_write(1,data)

# Ch B on 2nd MAX522 DAC chip.
def max522_dac2_chB(data):
	# if (data<25):
	# 	max522_dac2_write(16,0)
	# else:
	max522_dac2_write(2,data)

# Shutdown all MAX522 DAC chip outputs.
# Done by setting bits 4 and 5 on first Hex high.
# MAX522 datasheet page 8.
def max522_shutdown():
	max522_dac2_write(24,0)

###################################
########## Buttons Setup ##########
###################################

mode = DigitalInOut(board.IO33)
mode.direction = Direction.INPUT


###################################
########### Input Setup ###########
###################################

# ESP32 12bit / 4096
# Circuitpython reads in 65536 (16bit) as software

# ... .value
# 		357 gnd   to    52110 3.287 V
# 						--> seems only accurate to ~2.6 V ... do I recal for 2.5V across the potentiometers ? Yes, I think so.
# resistance ~ 4k from pots... 2k resistor on Vcc would accomplish that.

joystick_deadzone = 10000

x_in = analogio.AnalogIn(board.A2)
y_in = analogio.AnalogIn(board.A3)
pan_sp_in = analogio.AnalogIn(board.A4)
tilt_sp_in = analogio.AnalogIn(board.A5)
pan_damp_in = analogio.AnalogIn(board.A6)
tilt_damp_in = analogio.AnalogIn(board.A7)


input_vals = {}
def poll_inputs():
	input_vals['x'] = x_in.value - 32768
	input_vals['y'] = y_in.value - 32768
	input_vals['pan_sp'] = pan_sp_in.value
	input_vals['tilt_sp'] = tilt_sp_in.value
	input_vals['pan_damp'] = pan_damp_in.value
	input_vals['tilt_damp'] = tilt_damp_in.value


###################################
########## Output Setup ###########
###################################

# 3 states... if not input signals aren't close enough to 0 V / 3.3 V then turn off the speeds
x_out = DigitalInOut(board.IO44)	# Left high, Right gnd		--> middle point, turn off tilt_sp_out
y_out = DigitalInOut(board.IO43)	# Up high, Down gnd			--> middle point, turn off pan_sp_out
x_out.direction = Direction.OUTPUT
y_out.direction = Direction.OUTPUT

# pan_sp_out = analogio.AnalogOut(board.DAC1)
# tilt_sp_out = analogio.AnalogOut(board.DAC2)


pan_sp_out = 0
tilt_sp_out = 0
pan_damp_out = 0
tilt_damp_out = 0

dacs = {
	'pan_sp_out' : max522_dac1_chA,
	'tilt_sp_out' : max522_dac1_chB,
	'pan_damp_out' : max522_dac2_chA,
	'tilt_damp_out' : max522_dac2_chB
}

def set_outputs_manual():
	deadzone = joystick_deadzone
	abs_x = abs(input_vals['x'])
	abs_y = abs(input_vals['y'])

	# normalize [-1,1] to [0,1] for x and y axis of joystick
	x_out.value = ((abs_x / input_vals['x']) + 1) / 2		# 0 right, 1 left
	y_out.value = ((abs_y / input_vals['y']) + 1) / 2		# 0 down, 1 up


	# If the joystick has moved out of the deadzone, then proceed.
	if (abs_x > deadzone):
		print('past deadzone')
		# Equation derived from adc_test_2() data below. Linear fit in Matlab. 20220408
		# pan_sp_out.value = (input_vals['pan_sp'] * 1.38) - 1590
		pan_sp_out = (input_vals['pan_sp'] * 1.38) - 1590
		# Convert 16bit Circuitpython resolution down to 8bit Max522 DAC resolution
		pan_sp_out = pan_sp_out / 256

	# Otherwise assume the joystick is within the deadzone, and set zero out the speed.
	#	This is how I get around cramming a 3-state variable into a digital binary.
	else:
		# pan_sp_out.value = 0
		pan_sp_out = 0

	# Repeat for the y axis on the joystick
	if (abs_y > deadzone):
		# tilt_sp_out.value = (input_vals['tilt_sp'] * 1.38) - 1590
		tilt_sp_out = (input_vals['tilt_sp'] * 1.38) - 1590
		# Convert 16bit Circuitpython resolution down to 8bit Max522 DAC resolution
		tilt_sp_out = tilt_sp_out / 256
	else:
		# tilt_sp_out.value = 0
		tilt_sp_out = 0

	pan_damp_out = ((input_vals['pan_damp'] * 1.38) - 1590) / 256
	tilt_damp_out = ((input_vals['tilt_damp'] * 1.38) - 1590) / 256

	print(f'pan sp: %f, x_out: %d' %(pan_sp_out,x_out.value))
	# print(f'tilt sp: %f, y_out: %d' %(tilt_sp_out,y_out.value))

	dacs['pan_sp_out'](pan_sp_out)
	dacs['pan_damp_out'](pan_damp_out)
	dacs['tilt_sp_out'](tilt_sp_out)
	dacs['tilt_damp_out'](tilt_damp_out)



###################################
########## Helper Funcs ###########
###################################

# It's very likely this function can be called more than once depending on the quit conditions and where it was executed from.
# Therefore, try/except each deinit action. I don't care to see the error messages, I know it was previously deinit'd.
deinit_repository = [
	# i2c,
	mode,
	spi,cs_1,cs_2,
	x_in,y_in,pan_sp_in,tilt_sp_in,pan_damp_in,tilt_damp_in,
	x_out,y_out
	]
def deinit_all():
	for obj in deinit_repository:
		# print('deinitializing' + str(obj))
		try:
			obj.deinit()
		except:
			pass

# Clamps a value (n) to the range [minn,maxn]
def clamp_val(n, minn, maxn):
	return min(max(n, minn), maxn)


###################################
##### Putting it All Together #####
############ THE LOOP #############
###################################

# Variables in:
# - target position in degrees
# - Gains:
#	   - either a list of P, PI, or PID.
#		   - Must be in that order. If PD, use I of 0. ie (1,0,2) for Kp=1, Ki=0, Kd=2
#		   - If trailing values are 0, can be left off. ie (1,2) for Kp=1, Ki=2, Kd=0
#	   - or a single value of P
# - Length of time to sample, in seconds. Default of 3s.
# def position_control(target_position,gains,sample_time=3.0,min_drive=mot_min_drive):
# 	global mot_min_drive
# 	mot_min_drive = min_drive
# 	# tau = 2*pi
# 	# conversion_factor = 60 * (10**9) # 60 seconds * 10^9 ns/s

# 	# Limit the confines to +/- 180 degrees
# 	target_position = format_target(target_position)

# 	# Convert sample_time to nanoseconds.
# 	sample_time *= (10**9)



# 	######## PID INTAKE ########
# 	# Convert our times from ns to s; otherwise Ki and Kd become very weird and unintuitive scales.
# 	conversion_factor = 1 / (10**9) # 60 seconds * 10^9 ns/s

# 	# setup a zeroed out PID dictionary. This will get updated to intake arguments in a couple lines.
# 	pid = {
# 		'Kp' : 0.0,
# 		'Ki' : 0.0,
# 		'Kd' : 0.0
# 	}
# 	# Check if the input variable is a list.
# 	if (isinstance(gains,list)):
# 		# If it is a list, then make sure it's length is not more than 3
# 		if len(gains)>3:
# 			print("Error. Incorrect gains.")
# 			return 0
# 		# Append 0s until its length is 3.
# 		for i in range(len(gains),3):
# 			gains.append(0)
# 		# Populate the pid dictionary to the gains variable passed in
# 		for i,gain in enumerate(pid):
# 			pid[gain] = gains[i]
# 	# If not a list, then assign Kp to the number.
# 	else:
# 		pid['Kp'] = gains


# 	######## Initial Values ########

# 	encoder_data = []
# 	pwm_val,last_position,counter,last_error,i_term=0,0,0,0,0
# 	threshold = mot_threshold
# 	q=1
# 	pos = get_position()
# 	last_now = monotonic_ns()
# 	sleep(0.001)	# Just to get a difference between the 2 times, so that dt != 0
# 	now = monotonic_ns()
# 	start_time = now


# 	######## THE LOOP ########

# 	while(q):
# 		# Calculate Proportional Gain and clamp to +/- 1
# 		error = get_error(target_position - last_position)
# 		if abs(error) > threshold:
# 			p_term = pid['Kp'] * error
# 			i_term += pid['Ki'] * (now-last_now) * conversion_factor * error
# 			# try:
# 			d_term = pid['Kd'] * ((error-last_error)/((now-last_now) * conversion_factor))
# 			# print("p: %.2f, i: %.2f, d: %.2f, sum: %.2f" %(p_term,i_term,d_term,p_term+i_term+d_term))
# 			# except ZeroDivisionError:
# 			#   d_term = 0
# 			# abs_threshhold_check = abs(abs(error)-threshhold)
# 			# threshhold_check = max(0,min((abs(error)-threshhold),1))
# 			# print(threshhold_check)
# 			pwm_val = int((clamp_val((p_term+i_term+d_term), -65535, 65535))   ) #* threshhold_check)
# 			set_pwm(pwm_val)
# 		else:
# 			# set_pwm(0)
# 			i_term = 0
# 			mot_pwm.duty_cycle = 0

# 		if (len(encoder_data)>1):
# 			counter+=1
# 			# last_speed = calc_rpm(encoder_data[0],encoder_data[-1],counts)
# 			print('{"time": %d, "deg": %.2f, "error": %.2f, "pwm_theoretical": %.2f, "pwm_actual": %.2f, "dir": %d}' %((now-start_time), pos, error, pwm_val, mot_pwm.duty_cycle, mot_dir.value))
# 			encoder_data.pop(0)
# 		q = min(now - start_time - sample_time,0)
# 		last_now = now
# 		now = monotonic_ns()
# 		pos = get_position()
# 		encoder_data.append((pos,now))
# 		last_position = pos
# 		last_error = error
# 	pwm_val = 0
# 	mot_pwm.duty_cycle = 0
# 	print('{"time": %d, "deg": %.2f, "error": %.2f, "pwm_theoretical": %.2f, "pwm_actual": %.2f, "dir": %d}' %((now-start_time), pos, error, pwm_val, mot_pwm.duty_cycle, mot_dir.value))
# 	# print('total samples: %d' %counter)
# 	# print((now - start_time)/(10**9))
# 	# print()



# ###################################
# ######### Testing Section #########
# ###################################

def adc_test_1():
	global pan_sp_out, x_in
	set_val = 65535
	pan_sp_out.value = set_val
	while (x_in.value >= 52110):
		set_val -= 1
		pan_sp_out.value = set_val
		print(set_val,x_in.value)
	print(set_val,x_in.value)
	sleep(30)
#54227	2.617 V		no resistors
#65535	48674 	2.419V		2.8k + 800 ohm v divider


def adc_test_2():
	global pan_sp_out, x_in
	set_val = 65535
	pan_sp_out.value = set_val
	while (set_val >= 0):
		pan_sp_out.value = set_val
		print(f"%i, %i, %i" %(65536-set_val,set_val,x_in.value))
		set_val -= 1
	print(f"%i, %i, %i" %(65536-set_val,set_val,x_in.value))
	sleep(30)


# adc_test_1()

# adc_test_2()



# ###################################
# ######## Shutdown Section #########
# ###################################


# # Deinit all the GPIO. Runs automatically at the end.
def exit_program(source):
	print("Quitting program per %s." %source)
	# test_func_1()
	# test_func_2()

	max522_shutdown()
	deinit_all()

# If code fails or reaches end, execute exit_program() with argument 'atexit'.
atexit.register(exit_program,'atexit')



# ###################################
# ######## Startup Section ##########
# ###################################

import supervisor
import sys

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

# This is the default that runs when Ras Pico is plugged in, restarted, ctrl+s, etc.
if __name__ == '__main__':
	print("enter data")
	print(">>")

	while(1):
		poll_inputs()
		# sleep(1)
		set_outputs_manual()
		sleep(0.01)


# 	# json_data = {
# 	# 	'target':0,
# 	# 	'Kp':4000,
# 	# 	'Ki':0,
# 	# 	'Kd':0,
# 	# 	'runtime':1,
# 	# 	'min_drive':mot_min_drive
# 	# }
# 	# while(1):
# 	# 	if supervisor.runtime.serial_bytes_available:
# 	# 		# print('Serial input detected. Hit enter when finished.')
# 	# 		data = sys.stdin.readline()
# 	# 		try:
# 	# 			buffer_json = loads(data)
# 	# 			for key in buffer_json:
# 	# 				json_data[key] = buffer_json[key]
# 	# 			# print('json: ' + dumps(json_data))
# 	# 			# position_control(json_data['target'],4000,1)
# 	# 			position_control(json_data['target'],[json_data['Kp'],json_data['Ki'],json_data['Kd']],json_data['runtime'],json_data['min_drive'])
# 	# 		except:
# 	# 			print('DataType Error: input was not in correct json format.')
# 	# 			print(data)
# 	# 		#print(data)
# 	# 		sleep(0.1)
	
# 	# exit_program('__main__')



