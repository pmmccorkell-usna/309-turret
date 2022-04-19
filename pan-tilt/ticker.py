# Patrick McCorkell
# January 2022
# US Naval Academy
# Robotics and Control TSD
#

from time import monotonic_ns

class Interrupt_Controller():
	def __init__(self):
		self._armed = False
		self.min_delay = 0 # Don't allow negative numbers for the delay.

		# Where our tickers will get stored.
		self.tickers = {}

	# I use this function in all my classes.
	def clamp_val(self, n):
		return (max(n, self.min_delay))

	def default(self):
		print("LOG: No callback assigned to this timer.")

	# obj.interrupt(name='str', delay=int, function=func_object)
	def interrupt(self,**kwargs):

		# Setup default values.
		buffer = {
			'name' : 'timer1',
			'delay' : 1,		# seconds
			'function' : self.default
		}
		# Populate the buffer with keyword arguments.
		for arg in buffer:
			buffer[arg] = (kwargs.get(arg) or buffer[arg])

		# Enforce the limit(s).
		self.clamp_val(buffer['delay'])

		# Prevent runaway / endless recursion.
		# Don't permit adding more threads while threads are active.
		if (not self._armed):
			# Populate the tickers dictionary using the buffer.
			self.tickers[buffer['name']] = [buffer['delay'], buffer['function']]
		else:
			print("LOG: Error. Loops running.")

	def loop(self):
		self.arm()
		now = monotonic_ns()	# ns
		last_ran = {}			# dictionary to store when each ticker's callback was last executed.
		conversion = 10**9		# seconds <> nanoseconds

		# Populate the last_ran dictionary with low values to run at start.
		for k in self.tickers:
			last_ran[k] = now - (self.tickers[k][0]*10**9 + 1)

		# While armed, run the threads.
		while(self._armed):
			# Iterate through the tickers.
			for k,v in self.tickers.items():
				# Check if enough time has expired.
				if ((now - last_ran[k]) > (v[0]*conversion)):

					# Callback.
					v[1]()

					# Update the time callback was last executed.
					last_ran[k] = monotonic_ns()
			# Update the time.
			now = monotonic_ns()

	def pause(self):
		self._armed = False

	def arm(self):
		self._armed = True

	def remove_interrupt(self,timer_name):
		self.pause()
		# pop() is error-proof. If the key 'timer_name' doesn't exist, it just does nothing and keeps chugging.
		self.tickers.pop(timer_name,None)

	def remove_interrupt_all(self):
		self.pause()
		for key in self.tickers:
			self.remove_interrupt(key)




#####################################
#####################################
########## Example Section ##########
#####################################
#####################################


if __name__ == '__main__':
	from time import sleep
	print()
	print()
	print('running as main')

	# Some example variables to play around with
	i = 0
	max_val = 10

	# Instantiate the class.
	a = Interrupt_Controller()

	tic_time = monotonic_ns()
	def tic():
		global tic_time
		tic_time = (monotonic_ns() / 10**9)
	
	def toc():
		global tic_time
		toc_time = (monotonic_ns() / 10**9)
		print(str(toc_time - tic_time) + " seconds between tictoc")

	# Print the example variable in one thread.
	def thread_1():
		# toc()
		global i
		print(i)
		# tic()

	# Manipulate the example variable in this thread.
	def thread_2():
		# toc()
		global i
		i+=1
		print('iterate i')
		# tic()

	# Another random thread.
	def thread_3():
		toc()
		print('thread 3 interrupt')
		tic()

	# Another Another random thread.
	def thread_4():
		# toc()
		global i, max_val
		print('checking if i is over max_val')
		if i > max_val:
			a.pause()
			print("PAUSE PAUSE PAUSE")
		# tic()

	# This function gets executed in order, is blocking, is not multitasked or threaded.
	def setup():
		global i, max_val

		sleep(1)
		print("Setting up the threads")

		# Setup some threads to point at above functions.
		# Names are arbitrary. Just need some bookkeeping inside the Class.
		a.interrupt(name='timer1', delay=0.15, function=thread_1)
		a.interrupt(name='timer2', delay=1, function=thread_2)
		a.interrupt(name='timer3', delay=0.02, function=thread_3)
		a.interrupt(name='timer4', delay=0.4, function=thread_4)

		sleep(2)

		# Start the threads...  
		a.loop()
		# notice, this line is blocking.
		# Nothing progresses beyond here until thread_4 executes a "pause" on the "loop".

		print()
		print('back in setup')
		sleep(5)

		# Remove timer 3, because why not.
		a.remove_interrupt('timer3')

		# Add a useless interrupt with no callback.
		# This will annoyingly inform me there is no callback assigned.
		a.interrupt(name='timer5',delay=2)

		# Reset i
		i = 0
		# Change max_val to a higher #
		max_val = 99


		# Start the loop again
		a.loop()

		# Program ends.
		print("Aufwiedersehen")
	
	setup()

