from random import randint, choice
from math import log2

class Predictor:
	def __init__(self, btbSize, histSize, tagSize, fsmState, isGlobHist, isGlobTable, shared):
		# save parameters for self
		self.btbSize = btbSize
		self.histSize = histSize
		self.tagSize = tagSize
		self.fsmState = fsmState
		self.isGlobHist = isGlobHist
		self.isGlobTable = isGlobTable
		self.shared = shared
		
		## init function:
		# determine history
		if self.isGlobHist:
			self.globalHist = [0] * self.histSize
		else:
			self.histTable = [[0] * self.histSize] * self.btbSize
		
		# determine fsm table
		if self.isGlobTable:
			self.globalTable = [self.fsmState] * (2**self.histSize)
		else:
			self.fsmTable = []
			for i in range(self.btbSize):
				self.fsmTable.append([self.fsmState] * (2**self.histSize))
		
			# self.fsmTable = [[self.fsmState] * (2 * (2**self.histSize))] * self.btbSize
		
		# create valid biy array:
		self.validBitTable = [0] * self.btbSize
		
		# create tag table and target table
		self.tagTable = [0] * self.btbSize
		self.targetTable = [0] * self.btbSize
	
		self.flush_num = 0  # Machine flushes
		self.br_num = 0     # Number of branch instructions
		
		self.size = self.btbSize * ( 1 + self.tagSize + 30)
		#			valid bit		tag size		
		if self.isGlobHist:
			self.size += self.histSize
		else:
			self.size += self.histSize * self.btbSize
		if self.isGlobTable:
			self.size += (2 * (2**self.histSize))
		else:
			self.size += (2 * (2**self.histSize)) * self.btbSize
		# self.size -= 2
			
		# btbsize *( valid + tag + target + hist + 2*2**histSize) 
	
	def predict(self, pc, dest):
		# parse pc (tuple), dest is list of one element [-dest-]
		tag = pc[log(self.btbSize):log(self.btbSize) + self.tagSize]
		entryIndex = tup2int(pc[0:log(self.btbSize)])
		print(0, log(self.btbSize))
		# print("predict:", "entry index:", entryIndex, "tag:", tag)
	
		# compare tag
		# if compareList(self.tagTable[entryIndex], tag)
		
		if self.validBitTable[entryIndex] == 1 and tup2int(tag) != tup2int(self.tagTable[entryIndex]):
			# print("tag1")
			dest[0] = 4 * tup2int(pc) + 4
			return False
		elif self.validBitTable[entryIndex] == 0:
			# print("tag0")
			dest[0] = 4 * tup2int(pc) + 4
			return False
		
		# lookup history and get fsm index
		histEntry = 0 # called fsmIndex in cpp
		if self.isGlobHist:
			histEntry = self.globalHist
			fsmIndex = tup2int(self.globalHist)
		else:
			histEntry = self.histTable[entryIndex]
			fsmIndex = tup2int(self.histTable[entryIndex])
	
		# get fsm state
		if self.isGlobTable:
			# compute shared option
			if self.shared == LSB_SHARED:
				m = pc[0:self.histSize]
				h = histEntry
				fsmIndex = tup2int(listXor(m, h))
			elif self.shared == MID_SHARED:
				m = pc[14:self.histSize + 14]
				h = histEntry
				fsmIndex = tup2int(listXor(m, h))
				
			taken = self.globalTable[fsmIndex]
		else:# local fsm table
			# print("local fsm check")
			taken = self.fsmTable[entryIndex][fsmIndex]
		
		# prepare taken state
		if fsm2bool(taken):
			dest[0] = self.targetTable[entryIndex]
			# print("tag2")
		else:
			dest[0] = 4 * tup2int(pc) + 4
			# print("tag3")
		
		# print("taken:", taken)
		return fsm2bool(taken)
	
	def update(self, pc, targetPc, taken, pred_dst=None):
		# add a new entry to the btb anyways
		#	update fsm
		#	update history
		# targetPc - actual address to jump
		# pred_dst - predicted address (for the stats ?)
		
		self.br_num += 1
		
		if not taken:
			if tup2int(pc) * 4 + 4 != pred_dst:
				self.flush_num += 1
		else:
			
			if targetPc != pred_dst:
				self.flush_num += 1
		
		# print(">>", hex(targetPc), hex(pred_dst), taken, "<<")
		# if targetPc != pred_dst:# and taken:
			# self.flush_num += 1
		
		entryIndex = tup2int(pc[0:log(self.btbSize)])
		tag = pc[log(self.btbSize):log(self.btbSize) + self.tagSize]
		
		isSameTag = self.tagTable[entryIndex] == tag
		
		# if not isSameTag:
			# add this to the btb
		self.tagTable[entryIndex] = tag
		self.targetTable[entryIndex] = targetPc
		self.validBitTable[entryIndex] = 1
		
		
		# UPDATE HISTORY AND FSM
		if not self.isGlobHist:
			
			# history lookup
			histIndex = tup2int(self.histTable[entryIndex])

			
			# history update
			self.histTable[entryIndex] = shift(self.histTable[entryIndex], 1 if taken else 0)
			
			if not self.isGlobTable:
				self.fsmTable[entryIndex][histIndex] = updateFsm(self.fsmTable[entryIndex][histIndex], taken)
			else:
				self.globalTable[histIndex] = updateFsm(self.globalTable[histIndex], taken)
		
		else:
			# fsm update
			if self.shared == NOT_SHARED:
				histIndex = tup2int(self.globalHist)
			elif self.shared == LSB_SHARED:
				histIndex = tup2int(listXor(self.globalHist, pc[0: self.histSize]))
			elif self.shared == MID_SHARED:
				histIndex = tup2int(listXor(self.globalHist, pc[14: self.histSize + 14]))
			# print("hist index:", histIndex)
			# history update
			self.globalHist = shift(self.globalHist, 1 if taken else 0)
			
			if not self.isGlobTable:
				self.fsmTable[entryIndex][histIndex] = updateFsm(self.fsmTable[entryIndex][histIndex], taken)
			else:
				self.globalTable[histIndex] = updateFsm(self.globalTable[histIndex], taken)

	def strGlobalTable(self):
		string = "global fsm table: ["
		for i in range(len(self.globalTable)):
			string += "{0:02b}".format(self.globalTable[i])
			if not i == len(self.globalTable) - 1:
				string += ", "
		string += "]"
		return string
	def strLocalTable(self, index):
		string = "["
		for i in range((2**self.histSize)):
			string += state2str(self.fsmTable[index][i])
			if not i == ((2**self.histSize)) - 1:
				string += ", "
		string += "]"
		return string
	def __str__(self):
		spaceFormat = '{:15}'
		string = "---btb table---\n"
		string += "(" + spaceFormat.format("tag") + ", " + spaceFormat.format("target")
		if not self.isGlobHist:
			string += ", " + spaceFormat.format("history")
		if not self.isGlobTable:
			string += " ," + spaceFormat.format("fsm")
		string += ")\n"
		
		for i in range(self.btbSize):
			string += "(" + str(self.tagTable[i]) + ", " + str(self.targetTable[i])
			if not self.isGlobHist:
				string += ", " + str(self.histTable[i])
			if not self.isGlobTable:
				string += ", " + self.strLocalTable(i)
			string += ")\n"
		if self.isGlobHist:
			string += "global history reg: " + str(self.globalHist) + "\n"
		if self.isGlobTable:
			string += self.strGlobalTable() + "\n"
		return string
	def printStats(self):
		print("flush_num:", self.flush_num, "br_num:", self.br_num , "size:" ,self.size, "b")

NOT_SHARED = 0
LSB_SHARED = 1
MID_SHARED = 2

def tup2int(tup):
	"""if we do that to pc than for actual number multiply by 4"""
	res = 0
	for i in reversed(range(len(tup))):
		res += (2**i) * tup[i]
	return res
	
def pc2hex(pc):
	"""we expect 30 bits pc that last two lsb bits are zero"""
	return hex(4 * tup2int(pc))

def state2bool(state):
	if state in [0, 1]:
		return False
	if state in [2, 3]:
		return True

def fsm2bool(fsm):
		if fsm in [0, 1]:
			return False
		return True

def listXor(lst1, lst2):
	# print("xoring:")
	# print(lst1)
	# print(lst2)
	res = []
	for i in range(len(lst1)):
		res.append(lst1[i] ^ lst2[i])
	# print(res)
	return res

def state2str(state):
	if state == 0:
		return "SNT"
	if state == 1:
		return "WNT"
	if state == 2:
		return "WT"
	if state == 3:
		return "ST"

def compareList(lst1, lst2):
	if len(lst1) != len(lst2):
		return False
	for i in range(len(lst1)):
		if lst1[i] != lst2[i]:
			return False
	return True

def shift(lst, value):
	newList = lst.copy()
	newList[1:len(newList)] = lst[0:len(lst) - 1]
	newList[0] = value
	return newList
	
def updateFsm(currentState, taken):
	additive = 1 if taken else -1
	nextState = currentState + additive
	if nextState < 0:
		nextState = 0
	if nextState > 3:
		nextState = 3
	return nextState
		
def log(x):
	return int(log2(x))

def randVec(size):
	lst = [0] * size
	for i in range(size):
		lst[i] = randint(0,1)
	return lst

class btbEntry:
	def __init__(self):
		self.tag = 0
		self.target = 0
		self.hist = 0
		self.fsm = 0
	def __str__(self):
		if self.fsm:
			return str((self.tag, self.target, self.hist, self.fsm))
		return str((self.tag, self.target, self.hist))
	def __repr__(self):
		return str(self)

def hex2pc(h):
	pc = [0] * 30
	if h == 0:
		return pc
	while True:
		i = int(log(h))
		if i <= 1:
			break
		pc[i - 2] = 1
		h -= 2**i
		if h == 0:
			break
	# print(pc)
	return pc

# def pc2hex(pc):
	# dec = tup2int(pc)
	# r)

# print(tup2int([0, 1, 1])) = 6

BTBSIZE = 2
HISTORYSIZE = 8
TAGSIZE = 26
FSMSTATE = 1

choice([True, False])

# p = Predictor(2, 2, 26, 1, True, True, NOT_SHARED)

# print(p)
# predict -> update

if False:
	pc1 = [0] * 30
	pc1[0:log(BTBSIZE)] = [1,0,0]
	pc1[log(BTBSIZE):log(BTBSIZE) + TAGSIZE] = [0,1,0,1,0,0,0,0,0,0]
	print("pc1:", tup2int(pc1))
	
	pc2 = [0] * 30
	pc2[0:log(BTBSIZE)] = [0,1,0]
	pc2[log(BTBSIZE):log(BTBSIZE) + TAGSIZE] = [0,0,1,0,1,0,0,0,0,0]
	print("pc2:", tup2int(pc2))
	
	pc3 = [0] * 30
	pc3[0:log(BTBSIZE)] = [1,1,0]
	pc3[log(BTBSIZE):log(BTBSIZE) + TAGSIZE] = [0,1,1,1,1,0,0,0,0,0]
	print("pc3:", tup2int(pc3))
	
	p.update(pc1, "jacob", True)
	p.update(pc2, "mike", False)
	p.update(pc2, "mike", False)
	p.update(pc2, "mike", True)
	p.update(pc2, "mike", True)
	p.update(pc2, "mike", False)
	p.update(pc2, "mike", False)
	p.update(pc3, "fredy", True)
	
	print(p)
	
	
	DEST1 = [0]
	DEST2 = [0]
	DEST3 = [0]
	res1 = p.predict(pc1, DEST1)
	res2 = p.predict(pc2, DEST2)
	res3 = p.predict(pc3, DEST3)
	print(res1, DEST1)
	print(res2, DEST2)
	print(res3, DEST3)

if True: # test1
	p = Predictor(2, 2, 26, 1, False, False, NOT_SHARED)
	commands = []
	commands.append([hex2pc(0x1230), False, 0x12300])	# not exists -> not taken, update fsm
	commands.append([hex2pc(0x87654), True, 0x45678])	# not exists -> not taken, update fsm
	commands.append([hex2pc(0x1230), True, 0x12300])	# exists -> not taken cos fsm, update fsm WNT
	commands.append([hex2pc(0x87654), True, 0x45678])	# exists -> Takes
	commands.append([hex2pc(0x1230), False, 0x12300])
	commands.append([hex2pc(0x87654), False, 0x45678])
	commands.append([hex2pc(0x87654), False, 0x45678])
	commands.append([hex2pc(0x10c), True, 0x200])
	commands.append([hex2pc(0x87654), False, 0x45678])

if False: # test2
	p = Predictor(2, 2, 26, 1, True, True, NOT_SHARED)
	commands = []
	commands.append([hex2pc(0x1230), False, 0x12300])
	commands.append([hex2pc(0x87654), True, 0x45678])
	commands.append([hex2pc(0x1230), True, 0x12300])
	commands.append([hex2pc(0x87654), True, 0x45678]) 
	commands.append([hex2pc(0x1230), False, 0x12300])
	commands.append([hex2pc(0x87654), False, 0x45678])
	commands.append([hex2pc(0x87654), False, 0x45678])
	commands.append([hex2pc(0x10c), True, 0x200])	
	commands.append([hex2pc(0x87654), False, 0x45678])

if False: # test 3
	p = Predictor(2, 8, 26, 1, True, True, LSB_SHARED)
	commands = []
	commands.append([hex2pc(0x10), True, 0x20])
	commands.append([hex2pc(0x10), True, 0x20])
	commands.append([hex2pc(0x10), True, 0x20])
	commands.append([hex2pc(0x10), True, 0x20])
	commands.append([hex2pc(0x10), True, 0x20])
	commands.append([hex2pc(0x14), False, 0x24])
	commands.append([hex2pc(0x14), False, 0x24])
	commands.append([hex2pc(0x14), False, 0x24])
	commands.append([hex2pc(0x14), False, 0x24])
	commands.append([hex2pc(0x14), False, 0x24])

"""TEST 1:
2 2 26 1 local_history local_tables not_using_share
0x1230 N 0x12300
0x87654 T 0x45678
0x1230 T 0x12300
0x87654 T 0x45678
0x1230 N 0x12300
0x87654 N 0x45678
0x87654 N 0x45678
0x10c T 0x200
0x87654 N 0x45678

0x1230 N 0x1234
0x87654 N 0x87658
0x1230 N 0x1234
0x87654 N 0x87658
0x1230 N 0x1234
0x87654 N 0x87658
0x87654 N 0x87658
0x10c N 0x110
0x87654 N 0x87658
flush_num: 4, br_num: 9, size: 132b

"""

"""TEST 2:
2 2 26 1 global_history global_tables not_using_share
0x1230 N 0x12300
0x87654 T 0x45678
0x1230 T 0x12300
0x87654 T 0x45678
0x1230 N 0x12300
0x87654 N 0x45678
0x87654 N 0x45678
0x10c T 0x200
0x87654 N 0x45678

0x1230 N 0x1234
0x87654 N 0x87658
0x1230 N 0x1234
0x87654 N 0x87658
0x1230 T 0x12300
0x87654 N 0x87658
0x87654 N 0x87658
0x10c N 0x110
0x87654 N 0x87658
flush_num: 5, br_num: 9, size: 122b

"""

"""TEST 3:
2 8 26 1 global_history global_tables using_share_lsb
0x10 T 0x20		
0x10 T 0x20
0x10 T 0x20
0x10 T 0x20
0x10 T 0x20
0x14 N 0x24
0x14 N 0x24
0x14 N 0x24
0x14 N 0x24
0x14 N 0x24

0x10 N 0x14
0x10 N 0x14
0x10 N 0x14
0x10 N 0x14
0x10 N 0x14
0x14 N 0x18
0x14 N 0x18
0x14 N 0x18
0x14 N 0x18
0x14 N 0x18
flush_num: 5, br_num: 10, size: 632b

"""

for c in commands:
	dest = [0]
	res = p.predict(c[0], dest)
	# print("dest:", dest)
	print(pc2hex(c[0]), end = " ")
	p.update(c[0], c[2], c[1], dest[0])
	print(res, end = " ")
	print(hex(dest[0]))
	
	print(p)
p.printStats()
#TODO:
	# predict of no updated pc
	# valid bit array 






































