# Patrick McCorkell
# April 2022
# US Naval Academy
# Robotics and Control TSD
#

from time import sleep
import board
import busio
# import pulseio
import analogio
from digitalio import DigitalInOut, Direction
from math import copysign
from ticker import Interrupt_Controller
from adafruit_dotstar import DotStar

class EW309():
	def __init__(self):
		self.init_SPI()
		self.max522_shutdown() 		# kill DAC outputs before we do anything else.
		self.init_inputs()
		self.init_featherS2()
		self.init_outputs()
		# self.init_pwm()
		self.init_tickers()

		self.deinit_repository = [
			self.manual_mode,						# button
			self.spi,self.cs_1,self.cs_2, #self.i2c,	# spi and i2c buses
			# self.imu,								# BNO055 imu
			# self.x_pulse_in, self.y_pulse_in,		# pulseio inputs
			self.x_in,self.y_in,self.pan_sp_in,self.tilt_sp_in,self.pan_damp_in,self.tilt_damp_in,	# ADC readins
			self.x_out,self.y_out			# Digital out
		]


	def poll_inputs(self):
		self.input_vals['x'] = self.x_in.value - 32768
		self.input_vals['y'] = self.y_in.value - 32768
		self.input_vals['pan_sp'] = self.pan_sp_in.value
		self.input_vals['tilt_sp'] = self.tilt_sp_in.value
		self.input_vals['pan_damp'] = self.pan_damp_in.value
		self.input_vals['tilt_damp'] = self.tilt_damp_in.value
		self.led = self.manual_mode.value


	###################################
	######### PulseIO Section #########
	###################################
	# def init_pwm(self):
	# 	self.x_pulse_in = pulseio.PulseIn(board.IO17,4)
	# 	self.y_pulse_in = pulseio.PulseIn(board.IO18,4)

	# def convert_pwm(self):
	# 	# Check that there are pulses
	# 	x_command = 0
	# 	y_command = 0
	# 	if len(self.x_pulse_in):
	# 		# Read last pulse entry, subtract 1.5 ms, divide by 500ms resolution, and clamp to range [-1,1]
	# 		x_command = self.clamp_val((self.x_pulse_in[-1] - 1500) / 500,-1,1)
	# 	if len(self.y_pulse_in):
	# 		# Read last pulse entry, subtract 1.5 ms, divide by 500ms resolution, and clamp to range [-1,1]
	# 		y_command = self.clamp_val((self.y_pulse_in[-1] - 1500) / 500,-1,1)

	# 	# Set the normalized valeus.
	# 	self.set_outputs(x_command,y_command)

	# def pause_pwm(self):
	# 	self.x_pulse_in.pause()
	# 	self.y_pulse_in.pause()
	
	# def resume_pwm(self):
	# 	if self.x_pulse_in.paused:
	# 		self.x_pulse_in.resume()
	# 	if self.y_pulse_in.paused:
	# 		self.y_pulse_in.resume()

	####################################
	########## Ticker Section ##########
	####################################
	def init_tickers(self):
		self.quit = 0
		self.tickers = Interrupt_Controller()

	def start_controller(self,func_name,update_interval):
		self.tickers.remove_interrupt_all()
		self.set_outputs()
		self.tickers.interrupt(name='check_mode', delay=0.1,function=self.check_mode)
		self.tickers.interrupt(name='controller',delay=update_interval,function = func_name)
		self.tickers.loop()
		self.set_outputs()

	def check_mode(self):
		# True is manual joystick, False is auto controller
		if not self.manual_mode:
			self.set_outputs()
			self.tickers.pause()


	###################################
	############ IMU Setup ############
	###################################
	# def init_I2C(self):
	# 	self.i2c = busio.I2C(scl=board.SCL,sda=board.SDA)
	# 	if (self.i2c.try_lock()):
	# 		print("i2c addresses scanned:")
	# 		print(self.i2c.scan())
	# 		self.i2c.unlock()
	# 	else:
	# 		print("Cannot scan i2c; locked out.")
	# 	print()

	# def init_imu(self):
	# 	self.init_I2C()
	# 	self.imu = adafruit_bno055.BNO055_I2C(self.i2c)
	
	# def calstatus_imu(self):
	# 	return self.imu.calibration_status

	# def quaternion(self):
	# 	return self.imu.quaternion
	
	# def euler(self):
	# 	return self.imu.euler

	###################################
	########## Helper Funcs ###########
	###################################

	# It's very likely this function can be called more than once depending on the quit conditions and where it was executed from.
	# Therefore, try/except each deinit action. I don't care to see the error messages, I know it was previously deinit'd.
	def deinit_all(self):

		# Make sure we turn off the DACs before losing SPI.
		for _ in range(3):
			self.max522_shutdown()
			self.tickers.remove_interrupt_all()
			sleep(0.2)

		for obj in self.deinit_repository:
			# print('deinitializing' + str(obj))
			try:
				obj.deinit()
			except:
				pass

	# Clamps a value (n) to the range [minn,maxn]
	def clamp_val(self,n, minn, maxn):
		return min(max(n, minn), maxn)


	###################################
	########### Input Setup ###########
	###################################
	def init_inputs(self):

		# ESP32 12bit / 4096
		# Circuitpython reads in 65536 (16bit) as software

		# ... .value
		# 		357 gnd   to    52110 3.287 V
		# 						--> seems only accurate to ~2.6 V ... do I recal for 2.5V across the potentiometers ? Yes, I think so.
		# resistance ~ 4k from pots... 2k resistor on Vcc would accomplish that.

		self.joystick_deadzone = 10000

		self.x_in = analogio.AnalogIn(board.A2)
		self.y_in = analogio.AnalogIn(board.A3)
		self.pan_sp_in = analogio.AnalogIn(board.A4)
		self.tilt_sp_in = analogio.AnalogIn(board.A5)
		self.pan_damp_in = analogio.AnalogIn(board.A6)
		self.tilt_damp_in = analogio.AnalogIn(board.A7)

		self.input_vals = {}

		self.manual_mode = DigitalInOut(board.IO33)
		self.manual_mode.direction = Direction.INPUT

	def init_featherS2(self):
		# Init Blink LED
		self.led = DigitalInOut(board.LED)
		self.led.direction = Direction.OUTPUT

		# Init LDO2 Pin
		self.vout2 = DigitalInOut(board.LDO2)
		self.vout2.direction = Direction.OUTPUT
		self.vout2.value = False

		# Init APA102 dotstar stuff
		self.led_rgb = DotStar(board.APA102_SCK, board.APA102_MOSI, 1, brightness=0.8, auto_write=True)

	###################################
	########## Output Setup ###########
	###################################
	def init_outputs(self):
		# 3 states... if not input signals aren't close enough to 0 V / 3.3 V then turn off the speeds
		self.x_out = DigitalInOut(board.IO44)	# Left high, Right gnd		--> middle point, turn off tilt_sp_out
		self.y_out = DigitalInOut(board.IO43)	# Up high, Down gnd			--> middle point, turn off pan_sp_out
		self.x_out.direction = Direction.OUTPUT
		self.y_out.direction = Direction.OUTPUT

		# pan_sp_out = analogio.AnalogOut(board.DAC1)
		# tilt_sp_out = analogio.AnalogOut(board.DAC2)


		self.pan_sp_out = 0
		self.tilt_sp_out = 0
		self.pan_damp_out = 0
		self.tilt_damp_out = 0

		self.dacs = {
			'pan_sp_out' : self.max522_dac1_chA,
			'tilt_sp_out' : self.max522_dac1_chB,
			'pan_damp_out' : self.max522_dac2_chA,
			'tilt_damp_out' : self.max522_dac2_chB
		}

	# Inputs normalized to [-1,1]
	def set_outputs(self,pan_speed=0,tilt_speed=0,pan_damp=0,tilt_damp=0):
		#abs_pan_speed = max(abs(pan_speed),1)
		#x_dir = max(pan_speed / abs_pan_speed,0)
		x_dir = copysign(0.5,pan_speed) + 0.5
		x_sp = abs(int(pan_speed * 255))
		x_damp = int(pan_damp*255)

		y_dir = copysign(0.5,tilt_speed) + 0.5
		y_sp = abs(int(tilt_speed * 255))
		y_damp = int(tilt_damp * 255)

		# print(x_dir,x_sp,y_dir,y_sp)
		self.set_outputs_raw(x_sp,x_dir,x_damp,y_sp,y_dir,y_damp)


	# Inputs not normalized.
	def set_outputs_raw(self,pan_speed,pan_dir,pan_damp,tilt_speed,tilt_dir,tilt_damp):
		self.x_out.value = pan_dir
		self.y_out.value = tilt_dir
		self.dacs['pan_sp_out'](pan_speed)
		self.dacs['tilt_sp_out'](tilt_speed)
		self.dacs['pan_damp_out'](pan_damp)
		self.dacs['tilt_damp_out'](tilt_damp)
		r = int((pan_damp+tilt_damp)/2)
		g =  (pan_dir*128)+int(pan_speed/2)
		b = (tilt_dir*128)+int(tilt_speed /2)
		# print('r %d, g %d, b %d' %(r,g,b))
		self.led_rgb[0]=( r, g, b, 0.8)


	def set_outputs_manual(self):
		deadzone = self.joystick_deadzone
		abs_x = abs(self.input_vals['x'])
		abs_y = abs(self.input_vals['y'])

		# normalize [-1,1] to [0,1] for x and y axis of joystick
		self.x_out.value = ((abs_x / self.input_vals['x']) + 1) / 2		# 0 right, 1 left
		self.y_out.value = ((abs_y / self.input_vals['y']) + 1) / 2		# 0 down, 1 up


		# If the joystick has moved out of the deadzone, then proceed.
		if (abs_x > deadzone):
			# print('past deadzone')
			# Equation derived from adc_test_2() data below. Linear fit in Matlab. 20220408
			# pan_sp_out.value = (input_vals['pan_sp'] * 1.38) - 1590
			self.pan_sp_out = (self.input_vals['pan_sp'] * 1.38) - 1590
			# Convert 16bit Circuitpython resolution down to 8bit Max522 DAC resolution
			self.pan_sp_out = self.pan_sp_out / 256

		# Otherwise assume the joystick is within the deadzone, and set zero out the speed.
		#	This is how I get around cramming a 3-state variable into a digital binary.
		else:
			self.pan_sp_out = 0

		# Repeat for the y axis on the joystick
		if (abs_y > deadzone):
			# tilt_sp_out.value = (input_vals['tilt_sp'] * 1.38) - 1590
			self.tilt_sp_out = (self.input_vals['tilt_sp'] * 1.38) - 1590
			# Convert 16bit Circuitpython resolution down to 8bit Max522 DAC resolution
			self.tilt_sp_out = self.tilt_sp_out / 256
		else:
			self.tilt_sp_out = 0

		self.pan_damp_out = ((self.input_vals['pan_damp'] * 1.38) - 1590) / 256
		self.tilt_damp_out = ((self.input_vals['tilt_damp'] * 1.38) - 1590) / 256

		# print(f'pan sp: %f, x_out: %d' %(self.pan_sp_out,self.x_out.value))
		# print(f'tilt sp: %f, y_out: %d' %(tilt_sp_out,y_out.value))

		self.dacs['pan_sp_out'](self.pan_sp_out)
		self.dacs['pan_damp_out'](self.pan_damp_out)
		self.dacs['tilt_sp_out'](self.tilt_sp_out)
		self.dacs['tilt_damp_out'](self.tilt_damp_out)

		r = (self.x_out.value + self.y_out.value) * 128
		g =  self.clamp_val(self.pan_sp_out ,0,255)
		b = self.clamp_val(self.tilt_sp_out ,0,255)
		intensity = self.clamp_val(self.pan_sp_out + self.tilt_sp_out,0,1)
		# print('r %d, g %d, b %d' %(r,g,b))
		self.led_rgb[0]=( r, g, b, 0.8)



	###################################
	############ SPI Setup ############
	###################################
	def init_SPI(self):

		# Chip Select for 1st MAX-522 chip.
		self.cs_1 = DigitalInOut(board.IO1)
		self.cs_1.direction = Direction.OUTPUT
		self.cs_1.value = 1

		# Chip Select for 2nd MAX-522 chip.
		self.cs_2 = DigitalInOut(board.IO3)
		self.cs_2.direction = Direction.OUTPUT
		self.cs_2.value = 1

		# Setup SCK and MOSI on default pins.
		self.spi = busio.SPI(board.SCK, MOSI=board.MOSI)

		# Acquire the SPI bus.
		while not self.spi.try_lock():
			pass

		# Per Max522 DAC datasheet page 10: ph0, pol0, freq up to 500kHz.
		self.spi.configure(baudrate=500000, phase=0, polarity=0)

	# Write 2 8bit values over the SPI bus.
	# Per Max522 DAC datasheet page 8.
	def spi_write(self,val1,val2):
		# Ensure the data is clamped to 8 bits each before sending.
		val1 = int(self.clamp_val(val1,0,255))
		val2 = int(self.clamp_val(val2,0,255))
		# print('spi_write:')
		# print(val1,val2)
		# Send it.
		self.spi.write(bytes([val1,val2]))



	###################################
	########## MAX-522 Setup ##########
	###################################

	# 1st MAX522 DAC chip.
	def max522_dac1_write(self,chan,data):
		self.cs_1.value = 0
		self.spi_write(chan,data)
		self.cs_1.value = 1

	# Ch A on 1st MAX522 DAC chip.
	def max522_dac1_chA(self,data):
		self.max522_dac1_write(1,data)

	# Ch B on 1st MAX522 DAC chip.
	def max522_dac1_chB(self,data):
		self.max522_dac1_write(2,data)

	# 2nd MAX522 DAC chip
	def max522_dac2_write(self,chan,data):
		self.cs_2.value = 0
		self.spi_write(chan,data)
		self.cs_2.value = 1

	# Ch A on 2nd MAX522 DAC chip.
	def max522_dac2_chA(self,data):
		self.max522_dac2_write(1,data)

	# Ch B on 2nd MAX522 DAC chip.
	def max522_dac2_chB(self,data):
		self.max522_dac2_write(2,data)

	# Shutdown all MAX522 DAC chip outputs.
	# Done by setting bits 4 and 5 on first Hex high.
	# MAX522 datasheet page 8.
	def max522_shutdown(self):
		self.max522_dac2_write(24,0)
