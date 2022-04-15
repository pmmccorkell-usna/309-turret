# Patrick McCorkell
# April 2022
# US Naval Academy
# Robotics and Control TSD
#


import sys
import supervisor
from json import loads, dumps

class Read_Serial_JSON():
	def __init__(self, intake_dict):
		self.json_dict = intake_dict
	
	def read_serial(self):
		buffer_json = {}
		while supervisor.runtime.serial_bytes_available:
			buffer_serial = sys.stdin.readline()
			try:
				buffer_json = loads(buffer_serial)
				for key in buffer_json:
					self.json_dict[key] = buffer_json.get(key,None)
				# print('json: ' + dumps(json_dict))
			except:
				print('DataType Error: input was not in correct json format.')
				print(buffer_serial)
		return self.json_dict

	# Not necessary, but prevents some confusion.
	# Prefer to just use print() / print(f'') from code.py
	def write_serial(self, a_string):
		print(a_string)

	# Prevent buffer from building up.
	def clear_serial(self):
		junk = 0
		while supervisor.runtime.serial_bytes_available:
			junk = sys.stdin.readline()
		junk = 0
