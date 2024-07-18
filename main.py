import os, time, sys

def tobin(dec, nbits):
	b = bin(dec)[::-1][:-2][::-1]
	while len(b)<nbits:
		b='0'+b
	return b

def todec(b):
	d = 0
	for i in range(0,len(b),1):
		d += int(b[i])*2**(len(b)-1-i)
	return d

def CA2(dec, nbits):
	n = dec
	if dec < 0:
		dec *= -1
		b = tobin(dec, nbits)
		i = {'0':'1','1':'0'}
		ib = ''
		for bb in b:
			ib += i[bb]
		n = int(ib,2)+1
	return n



class Entity(object):
	def __init__(self, name, protect=False):
		self.id = hex(id(self))
		self.name = name
		self.protect = protect

# Wire is fully asynchronous
class Wire(Entity):
	def __init__(self, name, default_value=0, protect=False):
		super().__init__(name, protect=protect)
		self.value = default_value
	
	def run(self, clock_phase):
		pass
	
	def write(self, value):
		self.value = value
	
	def read(self):
		return self.value
	
	def getSignals(self):
		return ((self.name, self))

class Bus(Entity):
	def __init__(self, name, width):
		super().__init__(name)
		self.width = width
		self.datalines = {}
		for i in range(0,width,1):
			self.datalines[i] = Wire(self.name+'_D'+str(i))
	
	def write(self, value):
		b = tobin(value, self.width)
		for i in range(0,self.width,1):
			self.datalines[self.width-1-i].write(int(b[i]))
	
	def read(self):
		d = 0
		for i in range(0,self.width,1):
			d += self.datalines[self.width-1-i].read()*2**(self.width-1-i)
		return d
	
	def printout(self):
		st = self.name+" "
		L = len(st)
		for i in range(0,self.width,1):
			st += 'D'+str(self.width-1-i)+"|"
		st += '\n'
		for i in range(0,L,1):
			st += chr(32)
		for i in range(0,self.width,1):
			st += " "+str(self.datalines[self.width-1-i].read())+"|"
		print(st)

class Port(object):
	def __init__(self, bus_object, bus_slice, value_slice=None):
		self.bus_object = bus_object
		self.bus_slice = bus_slice
		self.value_slice = value_slice
		self.width = bus_slice[0]-bus_slice[1]+1
	
	def write(self, v):
		# if a 'value_slice' is specified, it means that we have to extract from the value, the slice corresponding to
		# 'value_slice' and then that value needs to be put on the bus on the bus_slice.
		# so for instance if I have an 8 bit bus and i write an 8 bit value to it but i want to extract bits 5:4 and put
		# them on the bus on bits 2:1, i can configure the two separate slices.
		value = v
		if self.value_slice != None:
			# slicing the initial value in order to extract only what we want
			value = tobin(v, 8)[::-1] # modified to 8
			value = int(value[self.value_slice[1]:self.value_slice[0]+1][::-1],2)
		# otherwise, no slicing on the value
		b = tobin(value, self.width)
		# now i have to apply padding bits to the slice
		for i in range(0,self.bus_slice[1],1):
			b += '0'
		for i in range(self.bus_slice[0]+1,self.bus_object.width,1):
			b = '0' + b
		# now i have to convert to dec
		d = int(b,2)
		# zeroing my bits
		p = ''
		for i in range(0, self.bus_object.width, 1):
			if i >= self.bus_slice[1] and i <= self.bus_slice[0]:
				p += '0'
			else:
				p += '1'
		p = p[::-1]
		p = int(p,2)
		self.bus_object.write((self.bus_object.read() & p) | d)
		
	def read(self):
		b = tobin(self.bus_object.read(), self.bus_object.width)
		b = b[::-1]
		return int(b[self.bus_slice[1]:self.bus_slice[0]+1][::-1], 2)


# A register is synchronous when it comes to latching in new data,
# it is asynchronous when it is reset or outputted.
class Register(Entity):
	def __init__(self, name, width, input_port_object, output_port_object, reset_obj, clock_obj, clock_phase, default_value, protect=False, cont_drive=None):
		super().__init__(name)
		self.width = width
		self.input_port = input_port_object
		self.output_ports = output_port_object
		self.cont_drive = cont_drive
		self.reset_wire = reset_obj
		self.clock_wire = clock_obj
		self.clock_phase = clock_phase
		# general properties
		self.value = default_value
		self.default_value = default_value
		# control wires
		self.signals = {}
		self.signals['DI'] = Wire(self.name+'_DI', protect=protect)
		self.signals['DO'] = Wire(self.name+'_DO', protect=protect)
	
	def run(self):
		if self.reset_wire.read()==1:
			# a reset is issued
			self.value = default_value
		else:
			# input is synchronous
			if self.signals['DI'].read()==1 and self.clock_wire.read()==self.clock_phase:
				# it is the right moment to input new data
				self.value = self.input_port.read()
			
			# output is asynchronous
			if self.signals['DO'].read()==1:
				for port_id in self.output_ports:
					self.output_ports[port_id].write(self.value)
			
			# checking for continuous drive outputs
			if self.cont_drive != None:
				for port_id in self.cont_drive:
					self.output_ports[port_id].write(self.value)
	
	def getSignals(self):
		list_of = []
		for sname in self.signals:
			if not self.signals[sname].protect:
				list_of.append((self.signals[sname].getSignals()))
		return list_of
	
	def printout(self):
		st = self.name+" "
		L = len(st)
		for i in range(0,self.width,1):
			st += 'B'+str(self.width-1-i)+"|"
		st += '\n'
		for i in range(0,L,1):
			st += chr(32)
		v = tobin(self.value, self.width)
		for i in range(0,self.width,1):
			st += " "+v[i]+"|"
		print(st)


class ProgramCounter(Entity):
	def __init__(self, name, cpu_type, input_port_object, output_port_object, reset_obj, clock_obj, clock_phase, default_value):
		super().__init__(name)
		self.input_port = input_port_object
		self.output_port = output_port_object
		self.reset_wire = reset_obj
		self.clock_wire = clock_obj
		self.clock_phase = clock_phase
		self.width = 0
		# general properties
		self.cpu_type = cpu_type
		self.value = default_value
		self.default_value = default_value
		# control wires
		self.signals = {}
		self.rbus = None
		self.rbus_port = None
		self.pc_limit = 0
		if self.cpu_type==0:
			# SAP-1 cpu
			self.width = 4
			self.rbus = Bus(self.name+'_rbus', 4)
			self.rbus_port = Port(self.rbus, (3,0), (3,0))
			self.rbus_port.write(0)
			self.pc_limit = 15
			self.signals['CP'] = Wire(self.name+'_CP')
			self.signals['EP'] = Wire(self.name+'_EP')
			self.signals['REG'] = Register(self.name+'_REG', 4, self.rbus_port, {'out':self.rbus_port}, self.reset_wire, self.clock_wire, self.clock_phase, self.default_value, protect=True)
		elif self.cpu_type==1:
			# Ben Eater - SAP-1 Hybrid CPU
			self.width = 4
			self.rbus = Bus(self.name+'_rbus', 4)
			self.rbus_port = Port(self.rbus, (3,0), (3,0))
			self.rbus_port.write(0)
			self.pc_limit = 15
			self.signals['CP'] = Wire(self.name+'_CP')
			self.signals['EP'] = Wire(self.name+'_EP')
			self.signals['IP'] = Wire(self.name+'_IP')
			self.signals['REG'] = Register(self.name+'_REG', 4, self.rbus_port, {'out':self.rbus_port}, self.reset_wire, self.clock_wire, self.clock_phase, self.default_value, protect=True)
		elif self.cpu_type==2:
			# SAP-2 Program Counter
			self.width = 16
			self.rbus = Bus(self.name+'_rbus', 16)
			self.rbus_port = {'LOW':Port(self.rbus, (7,0), (7,0)),'HIGH':Port(self.rbus, (15,8), (7,0))}
			self.rbus_port['LOW'].write(0)
			self.rbus_port['HIGH'].write(0)
			self.pc_limit = 2**16
			self.signals['CP'] = Wire(self.name+'_CP')
			self.signals['EP_L'] = Wire(self.name+'_EP_L')
			self.signals['EP_H'] = Wire(self.name+'_EP_H')
			self.signals['IP_L'] = Wire(self.name+'_IP_L')
			self.signals['IP_H'] = Wire(self.name+'_IP_H')
			self.signals['REG_L'] = Register(self.name+'_REG_L', 8, self.rbus_port['LOW'], {'out':self.rbus_port['LOW']}, self.reset_wire, self.clock_wire, self.clock_phase, self.default_value, protect=True)
			self.signals['REG_H'] = Register(self.name+'_REG_H', 8, self.rbus_port['HIGH'], {'out':self.rbus_port['HIGH']}, self.reset_wire, self.clock_wire, self.clock_phase, self.default_value, protect=True)
			
	
	def run(self):
		# checking reset
		if self.reset_wire.read()==1:
			# a reset was issued, so:
			if self.cpu_type < 2:
				self.signals['REG'].signals['DI'].write(0)
				self.signals['REG'].signals['DO'].write(0)
				self.rbus_port.write(0)
				self.signals['REG'].run()
			else:
				self.signals['REG_L'].signals['DI'].write(0)
				self.signals['REG_L'].signals['DO'].write(0)
				self.signals['REG_H'].signals['DI'].write(0)
				self.signals['REG_H'].signals['DO'].write(0)
				self.rbus_port['LOW'].write(0)
				self.rbus_port['HIGH'].write(0)
				self.signals['LOW'].run()
				self.signals['HIGH'].run()
		else:
			# STAGE 0 - PREPARING SIGNALS for INPUT AND OUTPUT
			# checking the pre-set
			if self.cpu_type < 2:
				if (self.signals['IP'].read()==1):
					if (self.clock_wire.read()==self.clock_phase):
						# jumping, so:
						jump_addr = self.input_port.read()
						self.signals['REG'].signals['DI'].write(1)
						self.rbus_port.write(jump_addr)
						self.value = jump_addr
				else:
					self.signals['REG'].signals['DI'].write(0)
			else:
				# checking input for the low 8 bit register
				if (self.signaàls['IP_L'].read()==1):
					if (self.clock_wire.read()==self.clock_phase):
						# updating
						pass
				else:
					self.signals['REG_L'].signals['DI'].write(0)
				# checking input for the high 8 bit register
				if (self.signals['IP_H'].read()==1):
					if (self.clock_wire.read()==self.clock_phase):
						# updating
						pass
				else:
					self.signals['REG_H'].signals['DI'].write(0)
			
			# normal operation, checking if it must increment
			if (self.signals['CP'].read()==1):
				# this means that the value stored in the register must increment by 1, so, first we get the current value
				if self.cpu_type==0:
					if self.clock_wire.read()==(self.clock_phase-1):
						reg_value = self.rbus_port.read()
						if reg_value==self.pc_limit:
							reg_value = 0
						else:
							reg_value += 1
						# now we have to update it, so we have to prepare the input for the next run
						self.signals['REG'].signals['DI'].write(1)
						self.rbus_port.write(reg_value)
						# now when 'run' of the register will be called, the value will be updated
				elif self.cpu_type==1:
					if self.clock_wire.read()==self.clock_phase:
						reg_value = self.rbus_port.read()
						if reg_value==self.pc_limit:
							reg_value = 0
						else:
							reg_value += 1
						self.signals['REG'].signals['DI'].write(1)
						self.rbus_port.write(reg_value)
						self.value = reg_value
			else:
				if (self.signals['IP'].read()==0):
					self.signals['REG'].signals['DI'].write(0)

			# checking if it must drive the output port
			if self.signals['EP'].read()==1:
				# must drive the output port, so we have to prepare for it
				self.signals['REG'].signals['DO'].write(1)
			else:
				self.signals['REG'].signals['DO'].write(0)
								
			# STAGE 1 - EXECUTING
			# running it
			self.signals['REG'].run()
			
			# STAGE 2 - UPDATING THE OUTPUTS
			if (self.signals['EP'].read()==1) and (self.signals['IP'].read()==0):
				self.value = self.rbus_port.read()
				self.output_port.write(self.value)
	
	def getSignals(self):
		list_of = []
		for sname in self.signals:
			if not self.signals[sname].protect:
				sublist = self.signals[sname].getSignals()
				if type(sublist)==type(()):
					list_of.append(sublist)
				else:
					if len(sublist)>0:
						for i in sublist:
							list_of.append(i)
		return list_of
	
	def printout(self):
		st = self.name+" "
		L = len(st)
		for i in range(0,self.width,1):
			st += 'B'+str(self.width-1-i)+"|"
		st += '\n'
		for i in range(0,L,1):
			st += chr(32)
		v = tobin(self.value, self.width)
		for i in range(0,self.width,1):
			st += " "+v[i]+"|"
		print(st)


class Accumulator(Entity):
	def __init__(self, name, cpu_type, input_port_object, output_port_object, reset_obj, clock_obj, clock_phase, default_value):
		super().__init__(name)
		self.input_port = input_port_object
		self.output_ports = output_port_object
		self.reset_wire = reset_obj
		self.clock_wire = clock_obj
		self.clock_phase = clock_phase
		self.width = 0
		# general properties
		self.cpu_type = cpu_type
		self.value = default_value
		self.default_value = default_value
		# control wires
		self.signals = {}
		self.rbus = None
		self.rbus_port = None
		if self.cpu_type==0:
			# SAP-1 cpu
			self.width = 8
			self.rbus = Bus(self.name+'_rbus', 8)
			self.rbus_port = Port(self.rbus, (7,0), (7,0))
			self.rbus_port.write(0)
			self.signals['ACC_IN'] = Wire(self.name+'_IN', protect=False)
			self.signals['ACC_OUT'] = Wire(self.name+'_OUT', protect=False)
			self.signals['REG'] = Register(self.name+'_REG', 8, self.rbus_port, {'out':self.rbus_port}, self.reset_wire, self.clock_wire, self.clock_phase, self.default_value, protect=True)
	
	def run(self):
		# checking reset
		if self.reset_wire.read()==1:
			# a reset was issued, so:
			self.signals['REG'].signals['DI'].write(0)
			self.signals['REG'].signals['DO'].write(0)
			self.rbus_port.write(0)
			self.signals['REG'].run()
		else:
			# STAGE 0 - PREPARING SIGNALS for INPUT AND OUTPUT
			# normal operation, checking if it must increment
			if self.signals['ACC_IN'].read()==1:
				if self.clock_wire.read()==self.clock_phase:
					self.signals['REG'].signals['DI'].write(1)
					self.rbus_port.write(self.input_port.read())
					self.value = self.rbus_port.read()
					# now when 'run' of the register will be called, the value will be updated
			else:
				self.signals['REG'].signals['DI'].write(0)
			
			# one output port is always being driven with the value held by the register
			self.output_ports['out_alubus'].write(self.value)
			# debug: the problem is that 'self.value' stays at zero.
			
			# checking if it must drive the output port to the mai bus
			if self.signals['ACC_OUT'].read()==1:
				# must drive the output port, so we have to prepare for it
				self.signals['REG'].signals['DO'].write(1)
			else:
				self.signals['REG'].signals['DO'].write(0)
						
			# STAGE 1 - EXECUTING
			# running it
			self.signals['REG'].run()
			
			# STAGE 2 - UPDATING THE OUTPUTS
			if self.signals['ACC_OUT'].read()==1:
				self.value = self.rbus_port.read()
				self.output_ports['out_wbus'].write(self.value)
				self.output_ports['out_alubus'].write(self.value)
	
	def getSignals(self):
		list_of = []
		for sname in self.signals:
			if not self.signals[sname].protect:
				sublist = self.signals[sname].getSignals()
				if type(sublist)==type(()):
					list_of.append(sublist)
				else:
					if len(sublist)>0:
						for i in sublist:
							list_of.append(i)
		return list_of
	
	def printout(self):
		st = self.name+" "
		L = len(st)
		for i in range(0,self.width,1):
			st += 'B'+str(self.width-1-i)+"|"
		st += '\n'
		for i in range(0,L,1):
			st += chr(32)
		v = tobin(self.value, self.width)
		for i in range(0,self.width,1):
			st += " "+v[i]+"|"
		return st



class MemoryController(Entity):
	def __init__(self, name, cpu_type, input_port_object, output_port_object, reset_obj, clock_obj, clock_phase, default_value):
		super().__init__(name)
		self.input_port = input_port_object
		self.output_port = output_port_object
		self.reset_wire = reset_obj
		self.clock_wire = clock_obj
		self.clock_phase = clock_phase
		# general properties
		self.cpu_type = cpu_type
		self.value = default_value
		self.default_value = default_value
		self.width = 0
		self.memory_array = {}
		# control wires
		self.signals = {}
		self.abus = None
		self.abus_port = None
		self.mem_size = 0
		if self.cpu_type==0:
			# SAP-1 cpu
			self.abus = Bus(self.name+'_abus', 4)
			self.abus_port = Port(self.abus, (3,0), (3,0))
			self.mem_size = 16
			self.width = 4
			for i in range(self.mem_size):
				self.memory_array[i] = 0
			self.signals['MAR_IN'] = Wire(self.name+'_MAR_IN')
			self.signals['MEM_OUT'] = Wire(self.name+'_MEM_OUT')
			self.signals['REG'] = Register(self.name+'_REG', 4, self.abus_port, {'out':self.abus_port}, self.reset_wire, self.clock_wire, self.clock_phase, self.default_value, protect=True)
	
	def run(self):
		# checking reset
		if self.reset_wire.read()==1:
			# a reset was issued, so:
			self.signals['REG'].signals['DI'].write(0)
			self.signals['REG'].signals['DO'].write(0)
			self.signals['REG'].run()
		else:
			# STAGE 0 - PREPARING SIGNALS for INPUT AND OUTPUT
			if (self.signals['MAR_IN'].read()==1):
				# must prepare the register to accept a new address
				self.abus_port.write(self.input_port.read())
				self.signals['REG'].signals['DI'].write(1)
			else:
				self.signals['REG'].signals['DI'].write(0)
			
			if (self.signals['MEM_OUT'].read()==1):
				# need to get the current address pointer
				self.signals['REG'].signals['DO'].write(1)
			else:
				self.signals['REG'].signals['DO'].write(0)
			
			# STAGE 1 - EXECUTING
			# running it
			self.signals['REG'].run()
			
			# STAGE 2 - UPDATING THE OUTPUTS
			if (self.signals['MEM_OUT'].read()==1):
				# getting the pointer
				self.value = self.abus_port.read()
				self.output_port.write(self.memory_array[self.value])
	
	def getSignals(self):
		list_of = []
		for sname in self.signals:
			if not self.signals[sname].protect:
				sublist = self.signals[sname].getSignals()
				if type(sublist)==type(()):
					list_of.append(sublist)
				else:
					if len(sublist)>0:
						for i in sublist:
							list_of.append(i)
		return list_of
	
	def printout(self):
		st = self.name+" "
		L = len(st)
		for i in range(0,self.width,1):
			st += 'B'+str(self.width-1-i)+"|"
		st += '\n'
		for i in range(0,L,1):
			st += chr(32)
		v = tobin(self.value, self.width)
		for i in range(0,self.width,1):
			st += " "+v[i]+"|"
		print(st)
	
	def testProgram(self):
		# this program loads '0xAA' in the accumulator and then it outputs it
		self.memory_array[0] = (0x0 << 4) | (0x6)	# lda A
		self.memory_array[1] = (0x2 << 4) | (0x7)	# sub B
		self.memory_array[2] = (0xe << 4)		# output
		self.memory_array[3] = (0x3 << 4) | (0x1)	# if (A != 0) goto 1, else fall through
		self.memory_array[4] = (0xe)			# output
		self.memory_array[5] = (0xf << 4) | (0x0)	# hlt
		self.memory_array[6] = 0x80
		self.memory_array[7] = 0x01
		
		


class AdderSubtractor(Entity):
	def __init__(self, name, cpu_type, input_port_object, output_port_object, reset_obj, clock_obj, clock_phase, default_value):
		super().__init__(name)
		self.input_port = input_port_object
		self.output_port = output_port_object
		self.reset_wire = reset_obj
		self.clock_wire = clock_obj
		self.clock_phase = clock_phase
		# general properties
		self.cpu_type = cpu_type
		self.value = default_value
		self.default_value = default_value
		self.width = 0
		# control wires
		self.signals = {}
		self.abus = None
		self.abus_port = None
		if self.cpu_type==0:
			# SAP-1 cpu
			self.width = 8
			self.abus = Bus(self.name+'_abus', 8)
			self.abus_port = Port(self.abus, (7,0), (7,0))
			self.signals['ADDSUB'] = Wire(self.name+'_ADDSUB')
			self.signals['ALU_OUT'] = Wire(self.name+'_ALU_OUT')
			self.signals['REG'] = Register(self.name+'_REG', 8, self.abus_port, {'out':self.abus_port}, self.reset_wire, self.clock_wire, self.clock_phase, self.default_value, protect=True)

	def run(self):
		# checking reset
		if self.reset_wire.read()==1:
			# a reset was issued, so:
			self.signals['REG'].signals['DI'].write(0)
			self.signals['REG'].signals['DO'].write(0)
			self.signals['REG'].run()
		else:
			# Reading the input continuously
			input_val = self.input_port.read()
			val = 0
			# STAGE 0 - PREPARING SIGNALS for INPUT AND OUTPUT
			if (self.signals['ADDSUB'].read()==0):
				# must perform the sum -> the operands are inside the input port which is a 16 bit wide port
				# the high 8 bits are for the B register while the low 8 are for the accumulator
				val = (input_val & 0x00FF) + ((input_val & 0xFF00)>>8)
			else:
				# must perform the sub instead
				val = (input_val & 0x00FF) - ((input_val & 0xFF00)>>8)
				#print("SUB:",val)
			# setting as CA2 and writing to port
			self.value = CA2(val, self.width)
			#print(self.value, self.width)
			self.abus_port.write(self.value)
			
			if (self.signals['ALU_OUT'].read()==1):
				# needs to output the sum/sub result to bus
				self.signals['REG'].signals['DI'].write(0)
				self.signals['REG'].signals['DO'].write(1)
				#print("HERE, i have to save the operation:",self.value,input_val)
			else:
				self.signals['REG'].signals['DI'].write(1)
				self.signals['REG'].signals['DO'].write(0)
			
			# STAGE 1 - EXECUTING
			# running it
			self.signals['REG'].run()
			
			# STAGE 2 - UPDATING THE OUTPUTS
			if (self.signals['ALU_OUT'].read()==1):
				# getting the pointer
				self.value = self.abus_port.read()
				self.output_port.write(self.value)

	def getSignals(self):
		list_of = []
		for sname in self.signals:
			if not self.signals[sname].protect:
				sublist = self.signals[sname].getSignals()
				if type(sublist)==type(()):
					list_of.append(sublist)
				else:
					if len(sublist)>0:
						for i in sublist:
							list_of.append(i)
		return list_of
	
	def printout(self):
		st = self.name+" "
		L = len(st)
		for i in range(0,self.width,1):
			st += 'B'+str(self.width-1-i)+"|"
		st += '\n'
		for i in range(0,L,1):
			st += chr(32)
		v = tobin(self.value, self.width)
		for i in range(0,self.width,1):
			st += " "+v[i]+"|"
		print(st)

class ControlUnit(object):
	def __init__(self, cpu_type, cpu_frequency, clock, reset, flags_input_port=None):
		self.cpu_type = cpu_type
		self.tstates = 0
		self.fetch_states = 0
		self.control_words = {}
		self.iset_mnem = {}
		self.iset_opcode = {}
		self.signals = {'HLT': Wire('HLT')}
		self.subdevs = {}
		self.period = 1.0/cpu_frequency
		self.clock_wire = clock
		self.reset_wire = reset
		self.screen_cfg = {}
		self.flags_input_port = flags_input_port
		# now to generate according to the SAP version, so:
		if (self.cpu_type == 0 or self.cpu_type==1):
			# sap-1, so we need to fetch the instruction set
			self.tstates = 6
			if self.cpu_type==0:
				self.fetch_states = 3
				prfx = "./iset/sap-1/"
			else:
				self.fetch_states = 2
				prfx = "./iset/be-sap-1/"
			f = os.listdir(prfx)
			for ifile in f:
				a = open(prfx+ifile,"r")
				data = a.read().split("\n")
				a.close()
				# parsing the instruction set file
				parsed = []
				for line in data:
					if not line.startswith("#") and len(line)>0:
						parsed.append(line)
				# getting instruction name and opcodes
				iname = parsed[0].split("LABEL",1)[1].strip()
				icode = int(parsed[1].split("OPCODE",1)[1].strip(),16)
				# fetching timing states info
				self.iset_mnem[iname] = icode
				self.iset_opcode[icode] = iname
				self.control_words[icode] = {}
				for Tstep in range(2,len(parsed),1):
					if chr(32) in parsed[Tstep]:
						# specifying signals
						timing_idx, high_sigs = parsed[Tstep].split(chr(32),1)
						timing_idx = int(timing_idx.split("T",1)[1].strip())
						subsigs = high_sigs.strip().split(",")
						# now to keep also flag into mind: SIGN, ZERO -> 00, 01, 10
						self.control_words[icode][timing_idx] = []
						for high_sig in subsigs:
							signame = high_sig.strip()
							if len(signame)>0:
								self.control_words[icode][timing_idx].append(signame)
						# end of this loop
					else:
						# nothing to specify, so:
						timing_idx = int(parsed[Tstep].split("T",1)[1].strip())
						self.control_words[icode][timing_idx] = []					
				# end of Tstep loop
				print("cu: found instruction [",iname,hex(icode),"]")
			# end of instruction search
		# now ok
	
	def add(self, dev_obj):
		# add device signals to the control word
		sigs = dev_obj.getSignals()
		print("cu: adding subdev [",dev_obj.name,"@",dev_obj.id,"]")
		for v in sigs:
			self.signals[v[1].name] = v[1]
		self.subdevs[dev_obj.name] = dev_obj

	def printout(self):
		print("="*40)
		print("Control Unit Report")
		print("Cpu Type:",self.cpu_type)
		print("Instruction Set:",len(self.iset_mnem),"instructions.")
		for iname in self.iset_mnem:
			print(iname, hex(self.iset_mnem[iname]))
		print("Control signals:",len(self.signals),"signals.")
		s = ''
		for sname in self.signals:
			s += sname+','
		s = s[:-1]
		print(s)
	
	# printing out to terminal in nice way
	def screenout(self):
		# printing out the outside pattern
		#st = self.__clearScreen()
		st = "+"
		st += ("-"*78)
		st += "+\n"
		for i in range(0,25,1):
			st += '|'
			st += (" "*78)
			st += "|\n"
		st += '+'
		st += ("-"*78)
		st += '+'
		# printing now inside
		st += self.__setCursor(2,2)+"SAP-X Simulator"
		st += self.__setCursor(2,32)+"CPU TYPE: "+str(self.cpu_type)
		st += self.__setCursor(3,32)+"CLK STAT: "
		self.screen_cfg["CLKSTAT"] = (3,42)
		st += self.__setCursor(2,52)+"CPU FREQ: "+str(1.0/self.period)+" Hz"
		st += self.__setCursor(17,52)+"INSTRUCTION SET"
		row_count = 18
		col_count = 52
		for iname in self.iset_mnem:
			st += self.__setCursor(row_count,col_count)+iname+"("+str(hex(self.iset_mnem[iname]))+")"
			self.screen_cfg[iname] = (row_count,col_count)
			if col_count >= 60:
				col_count = 52
				row_count += 1
			else:
				col_count += 10
		st += self.__setCursor(3,2)+"SYSTEM MODULES"
		row_count = 4
		col_count = 2
		for mname in self.subdevs:
			st += self.__setCursor(row_count,col_count)+mname
			if col_count >= 22:
				col_count = 2
				row_count += 1
			else:
				col_count += 5
		# now the control signals
		st += self.__setCursor(7,2)+"CONTROL SIGNALS"
		row_count = 8
		col_count = 2
		for sname in self.signals:
			st += self.__setCursor(row_count,col_count)+sname
			self.screen_cfg[sname] = (row_count,col_count)
			if col_count >= 50:
				col_count = 2
				row_count += 1
			else:
				col_count += 15
		# timing states
		st += self.__setCursor(12,2)+"TIMING STATES"
		row_count = 12
		col_count = 17
		for i in range(0,6,1):
			self.screen_cfg['T'+str(i)] = (row_count,col_count)
			st += self.__setCursor(row_count,col_count)+"T"+str(i)
			col_count += 5
		st += self.__setCursor(12, 47)+"FETCH CYC."
		st += self.__setCursor(12, 62)+"EXECUTE CYC."
		self.screen_cfg['FETCH'] = (12,47)
		self.screen_cfg['EXEC'] = (12,62)
		# WBUS
		st += self.__setCursor(14, 2)+"WBUS"
		self.screen_cfg['WBUS'] = (15,2)
		# Accumulator
		st += self.__setCursor(14, 15)+"ACCUMULATOR"
		self.screen_cfg['AR'] = (15,15)
		st += self.__setCursor(14, 30)+"B REGISTER"
		self.screen_cfg['BR'] = (15,30)
		st += self.__setCursor(14,45)+"OUT REGISTER"
		self.screen_cfg['OR'] = (15,45)
		st += self.__setCursor(14,60)+"INSTR. REGISTER"
		self.screen_cfg['IR'] = (15,60)
		# clock pulses info
		st += self.__setCursor(3,52)+"CLK #: "
		self.screen_cfg['CLK#'] = (3,62)
		# running time info
		st += self.__setCursor(4,52)+"S. TIME: "
		self.screen_cfg['STIME'] = (4,62)
		st += self.__setCursor(5,52)+"R. TIME: "
		self.screen_cfg['RTIME'] = (5,62)
		# program counter and memory
		st += self.__setCursor(17,2)+"PROGRAM COUNTER"
		self.screen_cfg['PC'] = (18,2)
		st += self.__setCursor(17,30)+"SRAM"
		row = 18
		col = 30
		st += self.__setCursor(row,col)
		for ram_addr in range(0,16,1):
			st += self.__setCursor(row,col)+"@"+hex(ram_addr)+" "+hex(self.subdevs['MAR'].memory_array[ram_addr])
			self.screen_cfg['MEM'+str(ram_addr)] = (row,col)
			if ram_addr == 7:
				row = 18
				col += 10
			else:
				row += 1
		# flag register display
		if self.cpu_type==1:
			st += self.__setCursor(20,2)+"FLAG REGISTER"
			self.screen_cfg['FR'] = (21,2)
		
		# ending
		st += self.__setCursor(25,2)
		print(st)
	
	def displayCLKstat(self, stat):
		if stat==0:
			print(self.__setCursor(*self.screen_cfg["CLKSTAT"])+self.__setColor((0,255,0))+"RUN"+self.__resetAttr())
		elif stat==1:
			print(self.__setCursor(*self.screen_cfg["CLKSTAT"])+self.__setColor((255,255,0))+"STP"+self.__resetAttr())

	def displayOff(self):
		print(self.__setCursor(27,2))
	
	def displayClk(self, n):
		print(self.__setCursor(*self.screen_cfg['CLK#'])+str(n))
	
	def turnOnI(self, iname):
		print(self.__setCursor(*self.screen_cfg[iname])+self.__setColor((255,255,0))+iname+"("+hex(self.iset_mnem[iname])+")")
	
	def turnOffI(self, iname):
		print(self.__setCursor(*self.screen_cfg[iname])+self.__resetAttr()+iname+"("+hex(self.iset_mnem[iname])+")")
	
	def displayWBUS(self):
		print(self.__setCursor(*self.screen_cfg['WBUS'])+tobin(self.subdevs['MAR'].output_port.bus_object.read(),8))
	
	def displayReg(self,regname):
		print(self.__setCursor(*self.screen_cfg[regname])+tobin(self.subdevs[regname].value,8))
		if (regname=='PC'):
			v = self.subdevs[regname].value
			if v > 0:
				print(self.__setCursor(*self.screen_cfg['MEM'+str(v-1)])+self.__resetAttr()+"@"+hex(v-1)+" "+hex(self.subdevs['MAR'].memory_array[v-1]))
			print(self.__setCursor(*self.screen_cfg['MEM'+str(v)])+self.__setColor((255,255,0))+"@"+hex(v)+" "+hex(self.subdevs['MAR'].memory_array[v])+self.__resetAttr())
	
	def turnOnSignal(self, sname):
		print(self.__setCursor(*self.screen_cfg[sname])+self.__setColor((255,0,0))+sname+self.__resetAttr())
	
	def turnOffSignal(self, sname):
		print(self.__setCursor(*self.screen_cfg[sname])+self.__resetAttr()+sname)
	
	def turnOnT(self, t):
		if t <= (self.fetch_states-1):
			print(self.__setCursor(*self.screen_cfg['T'+str(t)])+self.__setColor((0,255,0))+"T"+str(t)+self.__resetAttr())
			print(self.__setCursor(*self.screen_cfg['FETCH'])+self.__setColor((0,255,0))+"FETCH CYC.")
			print(self.__setCursor(*self.screen_cfg['EXEC'])+self.__resetAttr()+"EXEC CYC.")
		else:
			print(self.__setCursor(*self.screen_cfg['T'+str(t)])+self.__setColor((255,0,0))+"T"+str(t)+self.__resetAttr())
			print(self.__setCursor(*self.screen_cfg['EXEC'])+self.__setColor((255,0,0))+"EXEC CYC.")
			print(self.__setCursor(*self.screen_cfg['FETCH'])+self.__resetAttr()+"FETCH CYC.")
	
	def turnOffT(self, t):
		print(self.__setCursor(*self.screen_cfg['T'+str(t)])+self.__resetAttr()+"T"+str(t))
		
	def __resetAttr(self):
		return chr(27)+"[0m"
	
	def __setColor(self, color):
		if color==(255,0,0):
			return chr(27)+"[97;101m"
		elif color==(0,255,0):
			return chr(27)+"[30;102m"
		elif color==(255,255,0):
			return chr(27)+"[30;103m"
		else:
			return chr(27)+"[48;2;"+str(color[0])+";"+str(color[1])+";"+str(color[2])+"m"
	
	def __clearScreen(self):
		return chr(27)+"[2J"
	
	def __setCursor(self, row, col):
		return chr(27)+"["+str(row)+";"+str(col)+"H"
	
	def displaySTIME(self, nclk):
		print(self.__setCursor(*self.screen_cfg['STIME'])+self.__resetAttr()+str(round(nclk*self.period,8)))
	
	def displayRTIME(self, t):
		print(self.__setCursor(*self.screen_cfg['RTIME'])+self.__resetAttr()+str(round(time.time()-t,4)))
	
	
	
	def runcpu(self):
		opcode = 0
		run = True
		nclk = 0
		#print("CPU STARTED")
		"""
		found reason for bug: fetch cycle for sap1 ben eater only 2.
		in this way i can implement the jump instr with the immediate operand
		"""
		self.screenout()
		tstart = time.time()
		flags_values = 0
		stat = int(sys.argv[3])
		seq = []
		while run:
			for timing_state in range(0, self.tstates, 1):
				# evaluating the flags
				if (self.cpu_type==1):
					flags_values = self.flags_input_port.read()
			
				# evaluating the timing state
				self.turnOnT(timing_state)
				if (self.cpu_type==0 or self.cpu_type==1):
					if timing_state <= (self.fetch_states-1):
						# FETCH CYCLE
						if timing_state==0:
							# setting the signals
							self.signals['PC_EP'].write(1)	# program counter out
							self.signals['MAR_MAR_IN'].write(1) # memory in
							self.turnOnSignal('PC_EP')
							self.turnOnSignal('MAR_MAR_IN')
						elif timing_state==1:
							# setting the signals
							self.signals['PC_EP'].write(0)
							self.signals['MAR_MAR_IN'].write(0)
							self.turnOffSignal('PC_EP')
							self.turnOffSignal('MAR_MAR_IN')
							self.signals['PC_CP'].write(1) # program counter + 1
							self.signals['MAR_MEM_OUT'].write(1) # memory out
							self.signals['IR_DI'].write(1) # instruction register in
							self.turnOnSignal('PC_CP')
							self.turnOnSignal('MAR_MEM_OUT')
							self.turnOnSignal('IR_DI')
						else:
							# setting the signals
							self.signals['PC_CP'].write(0)
							self.signals['MAR_MEM_OUT'].write(0)
							self.signals['IR_DI'].write(0)
							self.turnOffSignal('PC_CP')
							self.turnOffSignal('MAR_MEM_OUT')
							self.turnOffSignal('IR_DI')
							self.signals['IR_DO'].write(1) # instruction register out
							self.signals['MAR_MAR_IN'].write(1) # memory IN
							self.turnOnSignal('IR_DO')
							self.turnOnSignal('MAR_MAR_IN')
					else:
						# EXECUTION CYCLE
						timing_idx = (flags_values << 3) | timing_state
						# setting new signals
						for sname in self.control_words[opcode][timing_idx]:
							#print(timing_state,"\t","turning on signal:",sname)
							self.turnOnSignal(sname)
							self.signals[sname].write(1)
				# other cpus here
				
				# displaying wbus contents
				self.displayWBUS()
				self.displayReg('AR')
				self.displayReg('BR')
				self.displayReg('OR')
				self.displayReg('IR')
				self.displayClk(nclk)
				self.displayReg('PC')
				if self.cpu_type==1:
					self.displayReg('FR')
				
				# checking halt condition
				if self.signals['HLT'].read()==1:
					run = False
					break;

				# checking now the eventual keypress
				if stat==1:
					self.displayCLKstat(1)
					ind = input()
					if ind=="0":
						stat=0
				self.displayCLKstat(0)
				
				# running this timing state
				for clock_phase in range(0,4,1):
					self.clock_wire.write(clock_phase)
					for dev_name in self.subdevs:
						self.subdevs[dev_name].run()
					# waiting for the right amount
					time.sleep(self.period/4.)
				# going to next timing
				
				# checking if we got the opcode
				if (self.cpu_type==0 or self.cpu_type==1):
					if timing_state == (self.fetch_states-1):
						# we now have the opcode
						opcode = self.subdevs['IR'].output_ports['out_cbus'].read()
						self.turnOnI(self.iset_opcode[opcode])
						#print("Decoded instruction:",self.iset_opcode[opcode],hex(opcode))
						# turning off signals for the last fetch-stage
						if self.cpu_type==0:
							# SAP-1 cpu has 3 fetch states
							self.signals['IR_DO'].write(0) # instruction register out
							self.signals['MAR_MAR_IN'].write(0) # memory IN
							self.turnOffSignal('IR_DO')
							self.turnOffSignal('MAR_MAR_IN')
						elif self.cpu_type==1:
							# Ben Eater's version of SAP-1 has only 2 fetch states, hence
							self.signals['PC_CP'].write(0)
							self.signals['MAR_MEM_OUT'].write(0)
							self.signals['IR_DI'].write(0)
							self.turnOffSignal('PC_CP')
							self.turnOffSignal('MAR_MEM_OUT')
							self.turnOffSignal('IR_DI')
					elif timing_state == 5:
						self.turnOffI(self.iset_opcode[opcode])
						
				# counting
				nclk += 1

				# updating time info
				self.displaySTIME(nclk)
				self.displayRTIME(tstart)

				# resetting used signals
				if (self.cpu_type==0 or self.cpu_type==1):
					if timing_state >= self.fetch_states:
						timing_idx = (flags_values << 3) | timing_state
						for sname in self.control_words[opcode][timing_idx]:
							#print(timing_state+1,"\t","turning off signal:",sname)
							self.turnOffSignal(sname)
							self.signals[sname].write(0)
				
				# accessing the alubus
				#self.subdevs['AR'].output_ports['out_alubus'].bus_object.printout()
				self.turnOffT(timing_state)
				
		self.displayClk(nclk)
		self.displayOff()
				
		tstop = time.time()
		#print("CPU HALTED")
		#print("CPU ran for:",nclk,"clock periods.")
		#print("Program execution time:",nclk*self.period,"s")
		#print("Simulator time:",tstop-tstart)
		#self.subdevs['AR'].printout()
		#self.subdevs['BR'].printout()
		#self.subdevs['OR'].printout()

class FlagsRegister(Entity):
	def __init__(self, name, cpu_type, input_port_object, output_port_object, reset_obj, clock_obj, clock_phase, default_value):
		super().__init__(name)
		self.input_port = input_port_object
		self.output_port = output_port_object
		self.reset_wire = reset_obj
		self.clock_wire = clock_obj
		self.clock_phase = clock_phase
		self.width = 0
		# general properties
		self.cpu_type = cpu_type
		self.value = default_value
		self.default_value = default_value
		# control wires
		self.signals = {}
		self.rbus = None
		self.rbus_port = None
		if self.cpu_type==1:
			# Ben-Eater - SAP-1 cpu
			self.width = 2
			self.rbus = Bus(self.name+'_rbus', 2)
			self.rbus_port = Port(self.rbus, (1,0), (1,0))
			self.rbus_port.write(0)
			self.signals['FR_IN'] = Wire(self.name+'_IN', protect=False)
			self.signals['FR_OUT'] = Wire(self.name+'_OUT', protect=False)
			self.signals['REG'] = Register(self.name+'_REG', 2, self.rbus_port, {'out':self.rbus_port}, self.reset_wire, self.clock_wire, self.clock_phase, self.default_value, protect=True)
	
	def run(self):
		# checking reset
		if self.reset_wire.read()==1:
			# a reset was issued, so:
			self.signals['REG'].signals['DI'].write(0)
			self.signals['REG'].signals['DO'].write(1)
			self.rbus_port.write(0)
			self.signals['REG'].run()
		else:
			# STAGE 0 - PREPARING SIGNALS for INPUT AND OUTPUT
			# normal operation, checking if it must increment
			if self.signals['FR_IN'].read()==1:
				if self.clock_wire.read()==self.clock_phase:
					self.signals['REG'].signals['DI'].write(1)
					indata = self.input_port.read()
					# checking the flags -> SIGN_FLAG, ZERO_FLAG
					if indata <= 127:
						if indata==0:
							self.rbus_port.write(0x01)
						else:
							self.rbus_port.write(0x00)
					else:
						self.rbus_port.write(0x02)
					self.value = self.rbus_port.read()
					# now when 'run' of the register will be called, the value will be updated
			else:
				self.signals['REG'].signals['DI'].write(0)
			
			# one output port is always being driven with the value held by the register
			self.signals['REG'].signals['DO'].write(1)
			self.output_port.write(self.value)
						
			# STAGE 1 - EXECUTING
			# running it
			self.signals['REG'].run()
			
			# STAGE 2 - UPDATING THE OUTPUTS
			self.value = self.rbus_port.read()
			self.output_port.write(self.value)
	
	def getSignals(self):
		list_of = []
		for sname in self.signals:
			if not self.signals[sname].protect:
				sublist = self.signals[sname].getSignals()
				if type(sublist)==type(()):
					list_of.append(sublist)
				else:
					if len(sublist)>0:
						for i in sublist:
							list_of.append(i)
		return list_of
	
	def printout(self):
		st = self.name+" "
		L = len(st)
		for i in range(0,self.width,1):
			st += 'B'+str(self.width-1-i)+"|"
		st += '\n'
		for i in range(0,L,1):
			st += chr(32)
		v = tobin(self.value, self.width)
		for i in range(0,self.width,1):
			st += " "+v[i]+"|"
		return st

		

if __name__=='__main__':
	# generating master signals
	print(chr(27)+"[31;32m")
	input("Set the terminal to 80x27 and hit any key to continue...")
	print(chr(27)+"[0m]")
	clk = Wire('CLK', default_value=0)
	rst = Wire('RST')
	# checking the CPU type and building the system consequently
	if (sys.argv[1] == "SAP1" or sys.argv[1] == "BE-SAP1"):
		# generating the system bus
		wbus = Bus('WBUS', 8)
		cpu_type = 0
		
		# generating the flags register
		fbus = None
		FR_input_port = None
		FR_output_port = None
		FR = None
		if sys.argv[1]=="SAP1":
			cpu_type = 0
		elif sys.argv[1]=="BE-SAP1":
			fbus = Bus('FBUS',2)
			FR_input_port = Port(wbus, (7,0), (7,0))
			FR_output_port = Port(fbus, (1,0), (1,0))
			FR = FlagsRegister('FR', 1, FR_input_port, FR_output_port, rst, clk, 2, 0)
			cpu_type = 1
		
		# generating control unit
		cu = ControlUnit(cpu_type, float(sys.argv[2]), clk, rst, flags_input_port=FR_output_port)
		
		# adding the flag register
		if cpu_type==1:
			cu.add(FR)
		
		# generating the program counter and its tap on the bus -> it only affects bits 3:0 in SAP-1
		PC_input_port = None
		PC_port = Port(wbus, (3,0), (3,0))
		if cpu_type==1:
			PC_input_port = Port(wbus, (3,0), (3,0))
		PC = ProgramCounter('PC', cpu_type, PC_input_port, PC_port, rst, clk, 2, 0)
		cu.add(PC)
		
		# generating the MAR
		MAR_input_port = Port(wbus, (3,0), (3,0))
		MAR_output_port = Port(wbus, (7,0), (7,0))
		MAR = MemoryController('MAR', 0, MAR_input_port, MAR_output_port, rst, clk, 2, 0)
		cu.add(MAR)
		
		# generating the instruction register
		IR_input_port = Port(wbus, (7,0), (7,0))
		IR_output_port_bus = Port(wbus, (3,0), (3,0))	# this port writes the low 4 nibble word to the wbus (the address of the operand)
		cbus = Bus('CBUS',4)
		IR_output_port_control = Port(cbus, (3,0), (7,4))	# this port writes the high 4 nibble word to the cbus (the opcode of the instruction)
		IR = Register("IR", 8, IR_input_port, {'out_wbus':IR_output_port_bus, 'out_cbus':IR_output_port_control}, rst, clk, 2, 0, cont_drive={'out_cbus':True})
		cu.add(IR)
		
		# generating the accumulator and the B register
		alubus = Bus('ALUBUS',16)
		AR_input_port = Port(wbus, (7,0), (7,0))
		AR_output_port_bus = Port(wbus, (7,0), (7,0))
		AR_output_port_ALU = Port(alubus, (7,0), (7,0))
		BR_input_port = Port(wbus, (7,0), (7,0))
		BR_output_port_ALU = Port(alubus, (15,8), (7,0))	
		AR = Accumulator("AR", 0, AR_input_port, {'out_wbus':AR_output_port_bus, 'out_alubus':AR_output_port_ALU}, rst, clk, 2, 0)	
		BR = Register("BR", 8, BR_input_port, {'out_alubus':BR_output_port_ALU}, rst, clk, 2, 0)
		cu.add(AR)
		cu.add(BR)
		
		# generating the Adder/Subtractor unit
		ALU_input_port = Port(alubus, (15,0), (15,0))
		ALU_output_port = Port(wbus, (7,0), (7,0))
		ALU = AdderSubtractor('ALU', 0, ALU_input_port, ALU_output_port, rst, clk, 2, 0)
		cu.add(ALU)
		
		# generating the output register
		OR_input_port = Port(wbus, (7,0), (7,0))
		OR = Register("OR", 8, OR_input_port, None, rst, clk, 2, 0)
		cu.add(OR)
		
		# printing a report
		cu.printout()
		
		# placing a test program
		MAR.testProgram()
	
		# ready to start the cpu
		cu.runcpu()

