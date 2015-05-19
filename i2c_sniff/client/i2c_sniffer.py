import sys
import cmd
from serial import Serial



class I2C(cmd.Cmd):
	prompt = 'i2c> '

	def __init__(self, device, baudrate):
		self.s = Serial(device, baudrate)	
		cmd.Cmd.__init__(self)

	def do_send(self, command):
		"""
send [command]

	Sends commands to the I2C module
	Commands syntax is as follows:
	[ 		- start bit
	] 		- stop bit
	0xXX 	- the expected byte
	* 		- Overwrite the byte with a value
	
	Examples:
	[0x26 0x10 *0x30] 
	The i2c module will wait for a start bit, 0x26(Read from 0x26), then a 0x10. the following byte that the slave will be overwritten with 0x30.
	Then the i2c module will expect a stop bit. After a stop bit iwll be matched, the i2c module will start searching for the same sequnce again.

	Note:
	The sequence MUST start with a start bit ([)

		"""
		self.s.write(command + '\n')
		self.s.flush()
	
	def do_start(self, line):
		"""
		Start intercepting
		"""
		self.do_send('start')

	def do_stop(self, line):
		"""
		Stop intercepting
		"""
		self.do_send('stop')


def main(argv):
	if len(argv) != 3:
		print "Usuage:\n %s <COM DEVICE> <BAUDRATE>" % sys.argv[0]
		print "Example:\n %s COM10 9600" % sys.argv[0]
		sys.exit(1)

	I2C(sys.argv[1], int(sys.argv[2])).cmdloop()



if __name__ == "__main__":
	main(sys.argv)
