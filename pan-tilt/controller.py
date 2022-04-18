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

	pantilt.set_outputs(pan_speed,pan_dir,pan_damp,tilt_speed,tilt_dir,tilt_damp)


