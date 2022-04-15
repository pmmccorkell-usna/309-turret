print("Hello asdf!")
from time import monotonic_ns
import board
import busio
import adafruit_ds3502
from analogio import AnalogIn
import rotaryio
from math import pi
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

encoder_counts_per_rev = 1250
interrupt_encoder = 0.01 * (10**9)
interrupt_encoder_sample_time = 0.00005 * (10**9)	
interrupt_motor = 0.0002 * (10**9)
interrupt_digipot = 0.5 * (10**9)
default_list_length = 10000


def read_wiper(compare_val):
	val = val_in.value
	mot_pwm.duty_cycle = val
	print("read: %d raw, %.2f %%, %.2f Volts" %(val, val*100/65535,val*3.0/65535))
	print("error: %.2f%%" %((val*100/65535) - compare_val))
	print()

def calc_rpm(data_0,data_f,counts):
	conversion_factor = 60 * (10**9) # 60 seconds * 10^9 ns/s
	pos_diff = (data_f[0] - data_0[0]) / counts
	time_diff = (data_f[1] - data_0[1])
	return_val = conversion_factor * pos_diff / time_diff 
	return return_val

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

# Variables in:
# - target speed in rpm
# - Gains:
# 		- either a list of P, PI, or PID.
# 			- Must be in that order. If PD, use I of 0. ie (1,0,2) for Kp=1, Ki=0, Kd=2
#			- If trailing values are 0, can be left off. ie (1,2) for Kp=1, Ki=2, Kd=0
#		- or a single value of P
# - Voltage offset for the digital potentiometer
# - Length of time to sample, in seconds
def speed_control(target_speed,gains,Voffset=0,sample_time=1.0):
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


	inptVltge,last_speed,counter=0,0,0
	pos = enc.position
	q=1
	now = monotonic_ns()
	start_time = now
	# time_offset = start_time + sample_time

	while(q):
		# Calculate Proportional Gain and clamp to +/- 1
		inptVltge = clamp_val(pid['Kp'] * (target_speed - last_speed) + Voffset, -1, 1)
		# actualVltge = 
		digipot(inptVltge)

		if (len(encoder_data)>1):
			counter+=1
			last_speed = calc_rpm(encoder_data[0],encoder_data[-1],counts)
			print("%d, %.2f, %.2f, %.2f" %((now-start_time), ((pos % counts) * (tau/counts)), last_speed, inptVltge))
			encoder_data.pop(0)
		q = min(now - start_time - sample_time,0)
		now = monotonic_ns()
		pos = enc.position
		encoder_data.append((pos,now))
		
	print('total samples: %d' %counter)
	print((now - start_time)/(10**9))


def demo():
	global interrupt_encoder, interrupt_digipot, encoder_counts_per_rev
	interrupt_enc = interrupt_encoder
	interrupt_pot = interrupt_digipot
	counts = encoder_counts_per_rev

	pos = 0
	now = monotonic_ns()
	last_pot_execute,last_enc_execute = now,now

	while(1):
		for i in range(0,127+1):
			encoder_data=[]
			while (now - last_pot_execute <= interrupt_pot):
				encoder_data.append((enc.position,monotonic_ns()))
				j=0
				while ((now - last_enc_execute) <= interrupt_enc):
					j+=1
					pos = enc.position
					now = monotonic_ns()
				encoder_data.append((pos,now))
				last_enc_execute = now
			print("speed: %.2f rpm" %calc_rpm(encoder_data[0],encoder_data[-1],counts))
			print("angle: %.2f degrees" %((encoder_data[-1][0] % counts) * (360/counts)))
			print('set : '+str(i)+', percent: %.2f %%' %(i/127*100))
			ds3502.wiper = i
			read_wiper(i/127*100)
			now = monotonic_ns()
			last_pot_execute = now

			mot_dir.value = not mot_dir.value

		for i in range(1,127):
			encoder_data=[]
			while (now - last_pot_execute <= interrupt_pot):
				encoder_data.append((enc.position,monotonic_ns()))
				j=0
				while ((now - last_enc_execute) <= interrupt_enc):
					j+=1
					pos = enc.position
					now = monotonic_ns()
				encoder_data.append((pos,now))
				last_enc_execute = now
			print("speed: %.2f rpm" %calc_rpm(encoder_data[0],encoder_data[-1],counts))
			print("angle: %.2f degrees" %((encoder_data[-1][0] % counts) * (360/counts)))
			print('set : '+str(127-i)+', percent: %.2f %%' %((127-i)/127*100))
			ds3502.wiper = (127-i)
			read_wiper((127-i)/127*100)
			now = monotonic_ns()
			last_pot_execute = now

			mot_dir.value = not mot_dir.value


def exit_program():
	print("Quitting program.")
	mot_pwm.deinit()
	mot_dir.deinit()


atexit.register(exit_program)

# speed_control(50,[0.02,3,3])	# target speed, Kp, Voff, time
demo()
