print("Hello asdf!")
from time import monotonic_ns
import board
import busio
import adafruit_ds3502
from analogio import AnalogIn
import rotaryio
from math import pi, floor
import atexit
import pwmio
from digitalio import DigitalInOut, Direction

i2c = busio.I2C(scl=board.GP1,sda=board.GP0)
mot_pwm = pwmio.PWMOut(pin=board.GP16,frequency=25000,duty_cycle=0)

mot_dir = DigitalInOut(board.GP17)
mot_dir.direction = Direction.OUTPUT
mot_dir.value = 0		# 0 forward, 1 reverse


def scan_i2c():
	if (i2c.try_lock()):
		print("i2c addresses scanned:")
		print(i2c.scan())
		i2c.unlock()
	else:
		print("cannot scan i2c; locked out")
	print()
scan_i2c()

ds3502 = adafruit_ds3502.DS3502(i2c)
val_in = AnalogIn(board.A0)
enc = rotaryio.IncrementalEncoder(board.GP14,board.GP15,1)

encoder_counts_per_rev = 1250*4
interrupt_encoder = 0.01 * (10**9)
interrupt_encoder_sample_time = 0.00005 * (10**9)	
interrupt_motor = 0.0002 * (10**9)
interrupt_digipot = 0.5 * (10**9)
default_list_length = 10000


def read_wiper(compare_val):
	val = val_in.value
	# mot_pwm.duty_cycle = val
	print("read: %d raw, %.2f %%, %.2f Volts" %(val, val*100/65535,val*3.0/65535))
	print("error: %.2f%%" %((val*100/65535) - compare_val))
	print()

def calc_rpm(data_0,data_f,counts):
	conversion_factor = 60 * (10**9) # 60 seconds * 10^9 ns/s
	pos_diff = (data_f[0] - data_0[0]) / counts
	time_diff = (data_f[1] - data_0[1])
	try:
		return_val = conversion_factor * pos_diff / time_diff 
		return return_val
	except ZeroDivisionError:
		return 0

def create_empty_array(n):
	buffer = []
	for _ in range(n):
		buffer.append(0)
	return buffer

# Clamps a value (n) to the range [minn,maxn]
def clamp_val(n, minn, maxn):
	return min(max(n, minn), maxn)

# shift, scale, round to integer, clamp value to [0,127], and set the wiper of the digipot.
def digipot(val):
	return_val = clamp_val(round((val + 1) * 64), 0, 127)
	ds3502.wiper = return_val
	# return return_val

def set_pwm(gain):
	# dir = floor(gain/65535)
	# gain = abs(gain)
	mot_dir.value = floor(gain/65535)
	mot_pwm.duty_cycle = clamp_val(abs(gain),10000,65535)
	# print('set gain %0.2f' %gain)
	# return ((dir,gain))


# Variables in:
# - target speed in rpm
# - Gains:
# 		- either a list of P, PI, or PID.
# 			- Must be in that order. If PD, use I of 0. ie (1,0,2) for Kp=1, Ki=0, Kd=2
#			- If trailing values are 0, can be left off. ie (1,2) for Kp=1, Ki=2, Kd=0
#		- or a single value of P
# - Voltage offset for the digital potentiometer
# - Length of time to sample, in seconds
def position_control(target_position,gains,sample_time=3.0):
	global encoder_counts_per_rev, mot_dir, mot_pwm
	counts = encoder_counts_per_rev
	tau = 2*pi
	encoder_data = []
	conversion_factor = 60 * (10**9) # 60 seconds * 10^9 ns/s

	# _integral = 0.0
	# _error_previous = 0.0
	# _deadzone = tau / 

	# Convert sample_time to nanoseconds.
	sample_time *= (10**9)

	# setup a zeroed out PID dictionary. This will get updated to intake values in a couple lines.
	pid = {
		'Kp' : 0.0,
		'Ki' : 0.0,
		'Kd' : 0.0
	}
	# Check if the input variable is a list.
	if (isinstance(gains,list)):
		# If it is a list, then make sure it's length is not more than 3
		if len(gains)>3:
			print("Error. Incorrect gains.")
			return 0
		# Append 0s until its length is 3.
		for i in range(len(gains),3):
			gains.append(0)
		# Populate the pid dictionary to the gains variable passed in
		for i,gain in enumerate(pid):
			pid[gain] = gains[i]
	# If not a list, then assign Kp to the number.
	else:
		pid['Kp'] = gains


	pwm_val,last_position,counter,last_error=0,0,0,0
	threshhold = 1
	pos = enc.position
	q=1
	now = monotonic_ns()
	start_time,last_now = now,now
	# time_offset = start_time + sample_time

	while(q):
		# Calculate Proportional Gain and clamp to +/- 1
		error = target_position - last_position
		if abs(error) > threshhold:
			p_term = pid['Kp'] * error
			i_term = pid['Ki'] * (now-last_now) * error
			try:
				d_term = pid['Kd'] * ((error-last_error)/(now-last_now))
			except ZeroDivisionError:
				d_term = 0
			pwm_val = clamp_val((p_term+i_term+d_term), -65535, 65535)
			set_pwm(pwm_val)
		else:
			mot_pwm.duty_cycle = 0

		if (len(encoder_data)>1):
			counter+=1
			last_speed = calc_rpm(encoder_data[0],encoder_data[-1],counts)
			print("%d, deg: %.2f, error: %.2f, pwm_val_theoretical: %.2f, pwm_actual: %.2f" %((now-start_time), ((pos % counts) * (360/counts)), error, pwm_val, mot_pwm.duty_cycle))
			encoder_data.pop(0)
		q = min(now - start_time - sample_time,0)
		last_now = now
		now = monotonic_ns()
		pos = enc.position
		encoder_data.append((pos,now))
		last_position = ((pos % counts) * (360/counts))
		last_error = error
		
	print('total samples: %d' %counter)
	print((now - start_time)/(10**9))



def test_func_1():
	start=monotonic_ns()
	for n in range(-65535,65535+1):
		floor(n/65535)
	end=monotonic_ns()
	print('math.floor:')
	print((end-start)/ (10**9))


def test_func_2():
	start=monotonic_ns()
	for n in range(-65535,65535+1):
		np.floor(n/65535)
	end=monotonic_ns()
	print('np.floor:')
	print((end-start)/ (10**9))

def exit_program():
	print("Quitting program.")
	mot_pwm.duty_cycle=0
	mot_pwm.deinit()
	mot_dir.deinit()
	# test_func_1()
	# test_func_2()

atexit.register(exit_program)

position_control(90,20,1)
