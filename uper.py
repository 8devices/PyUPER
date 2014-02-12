#!/usr/bin/env python
# encoding: utf-8

__version__ = "0.02"

import struct, serial, threading, Queue, types, platform, glob
from up_pinmap import *

class Readers:
	def __init__(self, serial, outq, callback, decodefun):
		self.serial = serial
		self.outq = outq
		self.callback = callback
		self.alive = True
		self.thread_read = threading.Thread(target=self.reader)
		self.thread_read.setDaemon(1)
		self.thread_read.start()
		self.decodefun = decodefun

	def reader(self):
		while self.alive:
			try:
				data = self.serial.read(1)            #read one, blocking
				n = self.serial.inWaiting()            #look if there is more
				if n:
					data = data + self.serial.read(n)    #and get as much as possible
				if data:
					if data[3] == '\x08':
						interrupt = self.decodefun(data)
						callbackthread = threading.Thread(target=self.callback, args=[interrupt[1]])
						callbackthread.start()
					else:
						self.outq.put(data)
			except RuntimeError:
				print "UPER error: serial port reading problem!!!"
				break
		self.alive = False

	def stop(self):
		if self.alive:
			self.alive = False
			self.thread_read.join()


class Uper:
	def __init__(self, serial_port=None):
		ser = None
		if serial_port is None:
			my_platform = platform.system()
			if my_platform == "Windows":
				ports_list = []
				for i in range(256):
					try:
						ser = serial.Serial(i)
						ports_list.append('COM' + str(i + 1))
						ser.close()
					except serial.SerialException:
						pass
			elif my_platform == "Darwin":
				ports_list = glob.glob("/dev/tty.usbmodem*")
			elif my_platform == "Linux":
				ports_list = glob.glob("/dev/ttyACM*")
		for my_port in ports_list:
			try:
				port_to_try = serial.Serial(
					port=my_port,
					baudrate=230400, #virtual com port on USB is always max speed
					parity=serial.PARITY_ODD,
					stopbits=serial.STOPBITS_ONE,
					bytesize=serial.EIGHTBITS,
					timeout=0.1
				)
				port_to_try.write(self.encodeSFP(255, []))
				uper_response = port_to_try.read(1)    #read one, blocking
				n = port_to_try.inWaiting()        #look if there is more
				if n:
					uper_response = uper_response + port_to_try.read(n)
				if self.decodeSFP(uper_response)[0] == -1: # found port with UPER
					ser = port_to_try
					break
				port_to_try.close()
			except:
				#print "bum"
				pass
		if not ser:
			raise ValueError("UPER error: No UPER found on serial ports.")
		self.ser = ser
		self.ser.flush()
		self.outq = Queue.Queue()
		self.reader = Readers(self.ser, self.outq, self.internalCallBack, self.decodeSFP)
		self.pinstatus = []
		for pin in range(0, len(pins)):
			if isinstance(pins[pin], int):
				self.pinstatus.append([None, None])
				self.__setPrimary(pin)
				self.pinstatus[pin][1] = PRIMARY_PIN_INPUT
				self.pinMode(pin, DEFAULT_INPUT_MODE)
			else:
				self.pinstatus.append(-1)

		self.interrupts = [None] * UPER_HARD_INTERRUPTS
		self.callbackdict = {}
		self.SPI0running = False
		self.SPI1running = False

		self.PWMRunning = [0, 0]
		self.PWMPortPeriod = [20000, 20000]         #50Hz, 50Hz
		self.PWMPortMAX = [0xffff, 0xffffffff]      #maxuint16, maxuint32
		self.PWMPortLimit = [255, 255]
		self.PWMPortFunctions = [[50,51,52],[60,61,62]]
		self.pwmpins = []
		self.pwmpins.append([0,0,None])		#27	PWM0_0 - port, pin, duty
		self.pwmpins.append([0,1,None])		#28 PWM0_1
		self.pwmpins.append([0,2,None])		#34 PWM0_2
		self.pwmpins.append([1,0,None])		#10 PWM1_0
		self.pwmpins.append([1,1,None])		#39 PWM1_1
		self.pwmpins.append([1,2,None])		#3  PWM1_2
		self.devicename = "uper"
		self.version = __version__

	def getInfo(self):
		return self.devicename, self.version

	def stop(self):
		for i in range(7):
			self.detachInterrupt(i)
		self.reader.stop()
		self.ser.flush()
		self.ser.close()

	def encodeINT(self, intarg):
		if intarg < 64:
			return (chr(intarg))
		packedint = struct.pack('>I', intarg).lstrip('\x00')
		return (chr(0xc0 | (len(packedint) - 1)) + packedint)

	def encodeBYTES(self, bytestr):
		if len(bytestr) < 64:
			return (chr(0x40 | len(bytestr)) + bytestr)
		packedlen = struct.pack('>I', len(bytestr)).lstrip('\x00')
		if len(packedlen) == 1:
			return ('\xc4' + packedlen + bytestr)
		elif len(packedlen) == 2:
			return ('\xc5' + packedlen + bytestr)
		else:
			print "UPER error: API - too long string"

	def encodeSFP(self, command, args):
		functions = {types.StringType: self.encodeBYTES, types.IntType: self.encodeINT}
		SFPcommand = chr(command) + ''.join(functions[type(arg)](arg) for arg in args)
		SFPcommand = '\xd4' + struct.pack('>H', len(SFPcommand)) + SFPcommand
		return (SFPcommand)

	def decodeSFP(self, buffer):
		result = []
		if buffer[0:1] != '\xd4':
			return ( result )
		buflen = struct.unpack('>H', buffer[1:3])[0] + 3
		result.append(struct.unpack('b', buffer[3:4])[0])
		pointer = 4
		args = []
		while pointer < buflen:
			argtype = ord(buffer[pointer:pointer + 1])
			pointer += 1
			if argtype < 64:                    #short int
				args.append(argtype)
			elif argtype < 128:                    #short str
				arglen = argtype & 0x3f
				args.append(buffer[pointer:pointer + arglen])
				pointer += arglen
			else:
				arglen = argtype & 0x0f
				if arglen < 4:            #decoding integers
					if arglen == 0:
						args.append(ord(buffer[pointer:pointer + 1]))
					elif arglen == 1:
						args.append(struct.unpack('>H', buffer[pointer:pointer + 2])[0])
					elif arglen == 2:
						args.append(struct.unpack('>I', '\x00' + buffer[pointer:pointer + 3])[0])
					elif arglen == 3:
						args.append(struct.unpack('>I', buffer[pointer:pointer + 4])[0])
					pointer += arglen + 1
				else:
					if argtype == 0xc4:        #decoding strings
						arglen = ord(buffer[pointer:pointer + 1])
					elif argtype == 0xc5:
						arglen = struct.unpack('>H', buffer[pointer:pointer + 2])[0]
						pointer += 1
					else:
						print "UPER error: Decode - bad parameter type."
						return #gal raise exception reiketu?
					pointer += 1
					args.append(buffer[pointer:pointer + arglen])
					pointer += arglen
		result.append(args)
		return (result)

	def UPER_IO(self, ret, buf):
		self.ser.write(buf)
		data = []
		if ret != 0:
			try:
				data = self.outq.get(True, 1)
			except Queue.Empty:
				print "UPER error: empty serial port exception."
				pass
		return (data)

	def __setPrimary(self, pin):
		self.UPER_IO(0, self.encodeSFP(1, [pins[pin]]))

	def __setSecondary(self, pin):
		self.UPER_IO(0, self.encodeSFP(2, [pins[pin]]))

	def pinMode(self, pin, pinmode):
		if pinmode in [INPUT_HIGHZ, INPUT_PULLUP, INPUT_PULLDOWN, OUTPUT]:
			if pinmode != OUTPUT:
				self.pinstatus[pin][0] = pinmode
				self.pinstatus[pin][1] = PRIMARY_PIN_INPUT
			else:
				self.pinstatus[pin][1] = PRIMARY_PIN_OUTPUT
			self.UPER_IO(0, self.encodeSFP(3, [pins[pin], pinmode]))
		else:
			print "UPER error: pinMode - Illegal pinmode - ", pinmode

	def digitalWrite(self, pin, value):
		if self.pinstatus[pin][1] == SECONDARY_PIN:
			self.__setPrimary(pin)
		if self.pinstatus[pin][1] != PRIMARY_PIN_OUTPUT:
			self.pinMode(pin, OUTPUT)
		self.pinstatus[pin][1] = PRIMARY_PIN_OUTPUT
		self.UPER_IO(0, self.encodeSFP(4, [pins[pin], value]))

	def digitalRead(self, pin):
		if self.pinstatus[pin][1] == SECONDARY_PIN:
			self.__setPrimary(pin)
		if self.pinstatus[pin][1] != PRIMARY_PIN_INPUT:
			self.pinMode(pin, self.pinstatus[pin][0])
		self.pinstatus[pin][1] = PRIMARY_PIN_INPUT
		return (self.decodeSFP(self.UPER_IO(1, self.encodeSFP(5, [pins[pin]])))[1][1])

	def analogRead(self, pin):
		if pins[pin] in adcs:
			if self.pinstatus[pin][1] != SECONDARY_PIN:
				self.UPER_IO(0, self.encodeSFP(3, [pins[pin], DEFAULT_ADC_PINMODE]))
				self.__setSecondary(pin)
				self.pinstatus[pin][1] = SECONDARY_PIN
			adcPin = adcs.index(pins[pin])
			return (self.decodeSFP(self.UPER_IO(1, self.encodeSFP(10, [adcPin])))[1][1])
		else:
			print "UPER error: analogRead - " + str(pin) + " is not ADC pin, refer to pinmap"
			return False

	def attachInterrupt(self, pin, mode, callback):
		try:
			interruptID = self.interrupts.index(pin)
			self.UPER_IO(0, self.encodeSFP(7, [interruptID]))    #detach interrupt
		except ValueError:
			try:
				interruptID = self.interrupts.index(None)
				self.interrupts[interruptID] = pin
			except ValueError:
				return False #no free ID's left
		self.callbackdict[pin] = [mode, callback]
		self.UPER_IO(0, self.encodeSFP(6, [interruptID, pins[pin], mode]))
		return True

	def detachInterrupt(self, pin):
		try:
			interruptID = self.interrupts.index(pin)
		except ValueError:
			return False # no interupt on pin
		self.interrupts[interruptID] = None
		try:
			del self.callbackdict[pin]
		except KeyError:
			return False # no interupt on pin
		self.UPER_IO(0, self.encodeSFP(7, [interruptID]))
		return True

	def proportion(self,value,istart,istop,ostart,ostop) :
		return int(float(ostart) + (float(ostop) - float(ostart)) * ((float(value) - float(istart)) / (float(istop) - float(istart))))

	def setPwmLimit(self, limit, porttoset = None):
		if porttoset is None:
			portrange = [0,1]
		else:
			if porttoset not in [0,1]:
				print "UPER error: PWM ports must be 0 or 1."
				return False
			portrange = [porttoset]
		for port in portrange:
			if 0 <= limit <= self.PWMPortPeriod[port]:
				self.PWMPortLimit[port] = limit
			else:
				print "UPER error: PWM limit must be in range [0 .. ", self.PWMPortPeriod[port], + "]."
				return False
		return True

	def setPwmPeriod(self, period, porttoset = None):
		if porttoset is None:
			portrange = [0,1]
		else:
			if porttoset not in [0,1]:
				print "UPER error: PWM ports must be 0 or 1."
				return False
			portrange = [porttoset]
		for port in portrange:
			if 0 <= period <= self.PWMPortMAX[port]:
				self.PWMPortPeriod[port] = period
				if self.PWMRunning[port] > 0:
					self.UPER_IO(0, self.encodeSFP(self.PWMPortFunctions[port][0], [period]))
			else :
				print "UPER error! PWM period for port", port, "can be only between 0-", self.PWMPortMAX[port]
				return False
			return True

	def PwmWrite(self, pin, value, polarity = PWM_DEFAULT_POLARITY):
		try:
			pwmpin = pwms.index(pins[pin])
			pwmport = self.pwmpins[pwmpin][0]
			if value < 0 :
				print "UPER Warning: PWM value",value,"is negative, converted to 0."
				value = 0
			if value > self.PWMPortLimit[pwmport]:
				print "UPER Warning: PWM value",value,"is to big, converted to current limit - ", self.PWMPortLimit[pwmport]
				value = self.PWMPortLimit[pwmport]
			hightime = self.proportion(value, 0, self.PWMPortLimit[pwmport], self.PWMPortPeriod[pwmport], 0)
			if polarity == HIGH:
				hightime = self.PWMPortPeriod[pwmport] - hightime
			if self.pwmpins[pwmpin][2] is None:
				self.PWMRunning[pwmport] += 1
				if self.PWMRunning[pwmport] == 1:                # start the PWM port
					self.UPER_IO(0, self.encodeSFP(self.PWMPortFunctions[pwmport][0], [self.PWMPortPeriod[pwmport]]))
				if self.pinstatus[pin][1] != SECONDARY_PIN:          # set secondary
					self.__setSecondary(pin)
					self.pinstatus[pin][1] = SECONDARY_PIN
			self.UPER_IO(0, self.encodeSFP(self.PWMPortFunctions[pwmport][1], [self.pwmpins[pwmpin][1], hightime]))
			self.pwmpins[pwmpin][2] = hightime
			return True
		except ValueError:
			print "UPER error: Pin No", pin, " - is not PWM pin."
			return False

	def PwmStop(self, pin = None):
		if pin is None:
			for pinn in pwms:
				pwmpin = pwms.index(pinn)
				pwmport = self.pwmpins[pwmpin][0]
				if self.pwmpins[pwmpin][2] is not None:
					self.pwmpins[pwmpin][2] = None
					self.PWMRunning[pwmport] -= 1
					if self.PWMRunning[pwmport] == 0:
						self.UPER_IO(0, self.encodeSFP(self.PWMPortFunctions[pwmport][2], []))
		else:
			try:
				pwmpin = pwms.index(pins[pin])
				pwmport = self.pwmpins[pwmpin][0]
				if self.pwmpins[pwmpin][2] is not None:
					self.pwmpins[pwmpin][2] = None
					self.PWMRunning[pwmport] -= 1
					if self.PWMRunning[pwmport] == 0:
						self.UPER_IO(0, self.encodeSFP(self.PWMPortFunctions[pwmport][2], []))
			except ValueError:
				print "UPER error: Pin No", pin, " - is not PWM pin."
				return False
		return True

	def spi0_begin(self, divider = 1, mode = 0):
		if not self.SPI0running:
			self.SPI0running = True
			self.__setSecondary(spi0[0])
			self.__setSecondary(spi0[1])
			self.__setSecondary(spi0[2])
		self.UPER_IO(0, self.encodeSFP(20, [divider, mode]))

	def spi0_trans(self, data, respond = 1):
		if self.SPI0running:
			if respond:
				return(self.decodeSFP(self.UPER_IO(1, self.encodeSFP(21, [data, respond])))[1][0])
			else:
				self.UPER_IO(0, self.encodeSFP(21, [data, respond]))

	def spi0_end(self):
		if  self.SPI0running:
			self.SPI0running = False
			self.__setPrimary(spi0[0])
			self.__setPrimary(spi0[1])
			self.__setPrimary(spi0[2])
			self.UPER_IO(0, self.encodeSFP(22, []))

	def spi1_begin(self, divider = 1, mode = 0):
		if not self.SPI1running:
			self.SPI1running = True
			self.__setSecondary(spi1[0])
			self.__setSecondary(spi1[1])
			self.__setSecondary(spi1[2])
			self.UPER_IO(0, self.encodeSFP(30, [divider, mode]))

	def spi1_trans(self, data, respond = 1):
		if self.SPI1running:
			if respond:
				return(self.decodeSFP(self.UPER_IO(1, self.encodeSFP(31, [data, respond])))[1][0])
			else:
				self.UPER_IO(0, self.encodeSFP(31, [data, respond]))

	def spi1_end(self):
		if  self.SPI1running:
			self.SPI1running = False
			self.__setPrimary(spi1[0])
			self.__setPrimary(spi1[1])
			self.__setPrimary(spi1[2])
			self.UPER_IO(0, self.encodeSFP(32, []))

	def i2c_begin(self):
		self.UPER_IO(0, self.encodeSFP(40, []))

	def i2c_trans(self, address, writeData, readLength):
		if not readLength:

			self.UPER_IO(1, self.encodeSFP(41, [address, writeData, 0]))
		else:
			result = self.decodeSFP(self.UPER_IO(1, self.encodeSFP(41, [address, writeData, readLength])))
			if type(result[1][0]) == types.IntType:
				return '', result[1]
			else:
				return result[1][0], None

	def i2c_end(self):
		self.UPER_IO(0, self.encodeSFP( 42, []))

	def getDeviceInfo(self):
		device_info = []
		result = self.decodeSFP(self.UPER_IO(1, self.encodeSFP(255, [])))
		if result[0] != -1:
			print "UPER error: getDeviceInfo wrong code."
			return
		result = result[1]
		if result[0] >> 24 != 0x55: # 0x55 == 'U'
			print "UPER error: getDeviceInfo unknown device/firmware type"
			return
		device_info.append("UPER") # type
		device_info.append((result[0] & 0x00ff0000) >> 16) #fw major
		device_info.append(result[0] & 0x0000ffff) #fw minor
		device_info.append(result[1]) # 16 bytes long unique ID from UPER CPU
		device_info.append(result[2]) # UPER LPC CPU part number
		device_info.append(result[3]) # UPER LPC CPU bootload code version
		return(device_info)

	def internalCallBack(self, intdata):
		try:
			self.callbackdict[self.interrupts[intdata[0]]][1]()
		except:
			print "UPER error: internalCallBack problem."
		return