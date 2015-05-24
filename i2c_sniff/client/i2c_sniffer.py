import sys
import cmd
import struct
from serial import Serial



class I2C(cmd.Cmd):
	prompt = 'i2c> '

	def __init__(self, device, baudrate):
		self.s = Serial(device, baudrate)	
		cmd.Cmd.__init__(self)

	def _send_cmd(self, command):
		# Send size first, followed by cmd and \n.
		# size does NOT contain the 2 bytes of size, # and $
		raw_cmd = '#' + struct.pack('!H', len(command)) + command + '$'
		#print raw_cmd
		self.s.write(raw_cmd)
		self.s.flush()

	def _send_cmd_and_wait(self, command):
		self._send_cmd(command)
		res = self.s.readline()
		print ">>> ", res.replace('\n','')

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

		# Wait for an op
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

def main(argv):
	if len(argv) != 3:
		print "Usuage:\n %s <COM DEVICE> <BAUDRATE>" % sys.argv[0]
		print "Example:\n %s COM10 9600" % sys.argv[0]
		sys.exit(1)

	I2C(sys.argv[1], int(sys.argv[2])).cmdloop()



if __name__ == "__main__":
	main(sys.argv)


#[ 0x1A 0x3 ] [ 0x1B *0xaa *0xbb *0xcc *0xdd *0xee *0xff