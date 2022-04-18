from serial import Read_Serial_JSON
from time import sleep

json_data = {
# 	'target':0,
# 	'Kp':4000,
# 	'Ki':0,
# 	'Kd':0,
# 	'runtime':1,
# 	'min_drive':mot_min_drive
}

ser = Read_Serial_JSON(json_data)


def your_controller(imu_h,imu_p,imu_r):
	# Initialize all the variables you can control:
	pan_speed = 0		# [0,1]
	pan_damp = 0		# [0,1]

	tilt_speed = 0		# [0,1]
	tilt_damp = 0		# [0,1]

	# Maybe pull some PID values from matlab over serial ?
	some_characteristics_for_your_controller = ser.read_serial()

	print(imu_h,imu_p,imu_r)
	sleep(0.3)

	##############################################
	####################### THINGS HAPPEN HERE
	##############################################

	return pan_speed,tilt_speed,pan_damp,tilt_damp

