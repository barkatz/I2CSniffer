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

	def _send_cmd_and_wait(self, command):
		"""
		Sends a command and waits for a reply.
		The reply should end with \n
		"""
		self._send_cmd(command)
		res = self.s.readline()
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

	def do_sniff(self, sniff_mode):
		"""
sniff 

Starts sniff mode. ctrl+c to break
		"""

		res = self._send_cmd_and_wait("sniff on")		
		self._prompt("Turned sniffing on: " + res.replace('\n', ''))
		res = self._send_cmd_and_wait('start')
		self._prompt("Enabled: " + res.replace('\n', ''))

		
		#t = threading.Thread(target = self._handle_sniff)
		#t.start()
		# try:
		# 	while True:
		# 		pass
		# except KeyboardInterrupt, ki:
		# 	self._prompt("Stopping")
		current_count = -1
		buf = ''
		try:	
			while True:
				 c = self.s.read(1)

				 if (c != '#'):
				 	continue
				 s = self.s.read(5)
				 
				 if not s.endswith('$'):
				 	self._prompt("Out of sync: %s" % s)
				 	continue
				 t = s[0]
				 v = s[1]
				 seq = struct.unpack("H", s[2:4])[0]
				 if (current_count == -1):
				 	current_count = seq
				 else:
				 	if (current_count != seq - 1):
				 		print "Missed?"
				 current_count = seq
				 
				 if (t == 'B'):
				  	buf += hex(ord(v))
				 elif t == '+' or t == '-':
				 	buf += t + ' ' 
				 elif t == '[' or t == ']':
				 	buf += ' ' + t + ' '
				 

				 if t == ']':
					print buf
				 	buf = ''


		except KeyboardInterrupt, ki:
			self._prompt("Closing sniffer...")				  	


		res = self._send_cmd_and_wait("sniff off")

		# Flush pipe....
		buf = ''
		while not buf.endswith('OK\n'):
			buf += self.s.read(1)

		self._prompt("Waiting for thread to finish...")
		self._prompt("Done.")

			

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