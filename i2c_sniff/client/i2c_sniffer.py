import sys
import cmd
import struct
import threading
from serial import Serial



class I2C(cmd.Cmd):
	prompt = 'i2c> '

	def __init__(self, device, baudrate):
		self.s = Serial(device, baudrate)	
		cmd.Cmd.__init__(self)

	def _prompt(self, line):
		print ">>> ", line

	def _send_cmd(self, command):
		"""
		command format:
		#<SIZE(2)><command(SIZE)>$
		size does NOT contain the 2 bytes of size, # and $
		The starting # and trailing $ are for sync issues.
		"""
		raw_cmd = '#' + struct.pack('!H', len(command)) + command + '$'
		#print raw_cmd
		self.s.write(raw_cmd)
		self.s.flush()

	def _send_cmd_and_wait(self, command, prompt=True, errorprompt=True):
		"""
		Sends a command and waits for a reply.
		The reply should end with \n
		"""
		self._send_cmd(command)
		res = self.s.readline()
		if prompt or (errorprompt and 'ERROR' in res):
			self._prompt(res.replace('\n',''))

		return res

	def do_seq(self, seq):
		"""
seq [command]

	Sends a sequnce command to the I2C module
	sequnce syntax is as follows:
	[ 		- start bit
	] 		- stop bit
	0xXX 	- the expected byte
	* 		- Overwrite the byte with a value
	
	Examples:
	seq [0x26 0x10 *0x30] 
	The i2c module will wait for a start bit, 0x26(Read from 0x26), then a 0x10. the following byte that the slave will be overwritten with 0x30.
	Then the i2c module will expect a stop bit. After a stop bit iwll be matched, the i2c module will start searching for the same sequnce again.

	Note:
	The sequence MUST start with a start bit ([)
		"""
		self._send_cmd_and_wait("seq " + seq)


	def _is_a_number(self, arg):
		try:
			int(arg, 0)
			return True
		except:
			return False


	def do_sniff(self, sniff_mode):
		"""
sniff [on/off/get/loop] <size>

start  - starts sniff mode. <size> the buffer size on the device to use.
stop 	- stops sniff mode
get 	- get the sniff results.
loop 	- starts/gets results in a loop :)
		"""
		if (len(sniff_mode.split()) < 1):
			print "must suply an arg."
			return
		cmd = sniff_mode.split()[0]
		if cmd == 'start':
			if (len(sniff_mode.split()) < 2):
				size = 0x100
			else:
				size =  sniff_mode.split()[1]
				# Make sure this is a number:
				if not self._is_a_number(size):
					self._prompt("Size must be a number not '%s'" % size)
					return
			
			self._prompt("Turned sniffing on: ")
			res  = self._send_cmd_and_wait("sniff start %s" % size)		
			if 'ERROR' in res:
				return


			self._prompt("Enabling interrupts: ")
			res = self._send_cmd_and_wait('start')
			if 'ERROR' in res:
				return


			
			self._prompt("sniff get --> To get the result of the current sniff")
		
		elif cmd == 'stop':
			res = self._send_cmd_and_wait("sniff stop")		
		
		elif cmd == 'get':
			res = self._send_cmd_and_wait("sniff get", prompt = False)
			res = res.replace('OK\n', '')
			self._prompt(res.replace('[', '\n['))

		elif cmd == 'loop':
			if (len(sniff_mode.split()) < 2):
				size = 0x100
			else:
				size =  sniff_mode.split()[1]
				# Make sure this is a number:
				if not self._is_a_number(size):
					self._prompt("Size must be a number not '%s'" % size)
					return
			try:
				while True:
					res  = self._send_cmd_and_wait("sniff start %s" % size, prompt = False, errorprompt = True)
					if 'ERROR' in res:
						return

					res = self._send_cmd_and_wait('start', prompt = False, errorprompt = True)
					if 'ERROR' in res:
						return
					#raw_input('ok?')
					res = self._send_cmd_and_wait("sniff get", prompt = False, errorprompt = True)
					if 'ERROR' in res:
						return

					res = res.replace('OK\n', '')
					print (res.replace('[', '\n[')),

			except KeyboardInterrupt, ki:
				self._prompt("Stoping...")
				res = self._send_cmd_and_wait("sniff stop")
				import time
				time.sleep(1)
				# Emptying pipe...
				while self.s.inWaiting() != 0:
					time.sleep(0.01)
					print "--->", self.s.read(self.s.inWaiting())
				self._prompt("Done ^^")
		else:
			self._prompt('help sniff ^^')	





	def _handle_sniff(self):
		for i in xrange(1000):
			print self.s.read(1),
		


	def do_start(self, line):
		"""
		Start intercepting
		"""
		self._send_cmd_and_wait('start')

	def do_stop(self, line):
		"""
		Stop intercepting
		"""
		self._send_cmd_and_wait('stop')






#####################################################
#
# MAIN MODULE
#
######################################################
def main(argv):
	if len(argv) != 3:
		print "Usuage:\n %s <COM DEVICE> <BAUDRATE>" % sys.argv[0]
		print "Example:\n %s COM10 115200" % sys.argv[0]
		sys.exit(1)

	I2C(sys.argv[1], int(sys.argv[2])).cmdloop()



if __name__ == "__main__":
	main(sys.argv)


#[ 0x1A 0x3 ] [ 0x1B *0x120 --> Changes the compass.