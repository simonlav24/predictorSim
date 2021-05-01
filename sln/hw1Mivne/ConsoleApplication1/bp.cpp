/* 046267 Computer Architecture - Spring 2020 - HW #1 */
/* This file should hold your implementation of the predictor simulator */

#include "bp_api.h"
#include <iostream>
#include <bitset>
#include <math.h>

#define VALID_BIT_SIZE 1
#define USING_SHARE_LSB 1
#define USING_SHARE_MID 2
#define NOT_USING_SHARE 0
#define PC_SIZE 30

#define STRONGLY_NOT_TAKEN 0
#define WEAKLY_NOT_TAKEN 1
#define WEAKLY_TAKEN 2
#define STRONGLY_TAKEN 3
#define DEBUG if (debug) std::cout << "[DEBUG]" 

bool debug = false;
// global variables
int gBtbSize;
int gHistSize;
int gTagSize;
int gFsmState;
bool gIsGlobalHist;
bool gIsGlobalTable;
int gShared;

uint32_t* tagTable;
uint32_t* targetTable;

uint32_t globalHist;
uint32_t* histTable;

uint32_t* globalTable;
uint32_t** fsmTable;

bool* validBitTable;
SIM_stats stats;

// getRange: returns bitwise section of the num from start to end inclusive
uint32_t getRange(uint32_t num, int start, int end) {
	if (start > end) {
		std::cout << "range error" << std::endl;
		return 0;
	}
	num /= pow(2, start);
	uint32_t mask = pow(2, end + 1 - start) - 1;
	return num & mask;
}
// fsmState2bool: returns true <=> takenState is strongly or weakly taken
bool fsmState2bool(int takenState) {
	if (takenState == WEAKLY_NOT_TAKEN || takenState == STRONGLY_NOT_TAKEN)
		return false;
	return true;
}
// updafeFsm: returns fsm state after incremention or decremention
int updateFsm(int currentFsm, bool taken) {
	int additive = taken ? 1 : -1;
	int nextFsm = currentFsm + additive;
	if (nextFsm < 0)
		nextFsm = 0;
	if (nextFsm > 3)
		nextFsm = 3;
	//DEBUG << "update fsm, taken: " << taken << " prevFSM: " << currentFsm << " nextFSM: " << nextFsm << std::endl;
	return nextFsm;

}

void printP() {
	if (!debug)
		return;
	std::cout << std::endl;
	std::cout << "-------Predictor--------" << std::endl;
	std::cout << "tag table:" << std::endl;
	for (int i = 0; i < gBtbSize; i++) {
		std::cout << tagTable[i] << " ";
	}
	std::cout << std::endl;

	if (!gIsGlobalHist) {
		std::cout << "hist table:" << std::endl;
		for (int i = 0; i < gBtbSize; i++) {
			std::cout << histTable[i] << " ";
		}
		std::cout << std::endl;
	}
	else {
		std::cout << "global hist:" << globalHist << std::endl;
	}

	if (!gIsGlobalTable) {
		std::cout << "fsm table:" << std::endl;
		for (int i = 0; i < gBtbSize; i++) {
			std::cout << i << ": ";
			for (int j = 0; j < pow(2, gHistSize); j++) {
				std::cout << fsmTable[i][j] << " ";
			}
			std::cout << std::endl;
		}
		std::cout << std::endl;
	}
	else {
		std::cout << "global fsm table:" << std::endl;
		for (int i = 0; i < pow(2, gHistSize); i++) {
			std::cout << globalTable[i] << " ";
		}
		std::cout << std::endl;
	}
	std::cout << "------------------" << std::endl;
}

int BP_init(unsigned btbSize, unsigned historySize, unsigned tagSize, unsigned fsmState,
			bool isGlobalHist, bool isGlobalTable, int Shared){
	
	gBtbSize = btbSize;
	gHistSize = historySize;
	gTagSize = tagSize;
	gFsmState = fsmState;
	gIsGlobalHist = isGlobalHist;
	gIsGlobalTable = isGlobalTable;
	gShared = Shared;

	/// alocate history table:
	if (!gIsGlobalHist) {
		histTable = new uint32_t[gBtbSize];
		for (int i = 0; i < gBtbSize; i++)
			histTable[i] = 0;
	}

	/// allocate fsm table:
	if (gIsGlobalTable) {
		globalTable = new uint32_t[pow(2, gHistSize)];
		for (int i = 0; i < pow(2, gHistSize); i++)
			globalTable[i] = gFsmState;
	}

	else
	{
		fsmTable = new uint32_t * [gBtbSize];
		for (int i = 0; i < gBtbSize; i++) {
			fsmTable[i] = new uint32_t[pow(2, gHistSize)];
			for (int j = 0; j < pow(2, gHistSize); j++) {
				fsmTable[i][j] = gFsmState;
			}
		}
	}

	/// allocate valid bit table:
	validBitTable = new bool[gBtbSize];
	for (int i = 0; i < gBtbSize; i++) {
		validBitTable[i] = false;
	}
	
	/// allocate tag table and target table
	tagTable = new uint32_t[gBtbSize];
	targetTable = new uint32_t[gBtbSize];

	// init stats
	stats.br_num = 0;
	stats.flush_num = 0;

	// calculate size:
	stats.size = gBtbSize * (VALID_BIT_SIZE + gTagSize + PC_SIZE);
	if (gIsGlobalHist)
		stats.size += gHistSize;
	else
		stats.size += gHistSize * gBtbSize;
	if (gIsGlobalTable)
		stats.size += 2 * pow(2, gHistSize);
	else
		stats.size += 2 * pow(2, gHistSize) * gBtbSize;
	 
	// testing area

	return 0;
}

bool BP_predict(uint32_t pc, uint32_t *dst){

	uint32_t tag = getRange(pc, 2 + log2(gBtbSize), 1 + log2(gBtbSize) + gTagSize);
	uint32_t entryIndex = getRange(pc, 2, 1 + log2(gBtbSize));// bug fixed 2 -> 1

	DEBUG << "######### predict:" << std::endl;
	DEBUG << "pc    = " << std::bitset<32>(pc) << std::endl;
	DEBUG << "tag   = " << std::bitset<32>(tag) << std::endl;
	DEBUG << "entry = " << std::bitset<32>(entryIndex) << std::endl;
	DEBUG << "histSize " << gHistSize << std::endl;
	if (validBitTable[entryIndex] == true and tag != tagTable[entryIndex]) {
		*dst = pc + 4;
		return false;
	}
	else if (validBitTable[entryIndex] == false) {
		*dst = pc + 4;
		return false;
	}

	// lookup history and get fsm index
	uint32_t fsmIndex = 0;
	if (gIsGlobalHist) {
		fsmIndex = globalHist;
	}
	else {
		fsmIndex = histTable[entryIndex];
	}
	DEBUG << "fsmIndex in predict = " << fsmIndex << std::endl;
	// get fsm state
	int takenState;
	if (gIsGlobalTable) {
		if (gShared == USING_SHARE_LSB) {
			uint32_t m = getRange(pc, 2, 2 + gHistSize);
			uint32_t h = fsmIndex;
			fsmIndex = m ^ h;
		}
		else if (gShared == USING_SHARE_MID) {
			uint32_t m = getRange(pc, 16, 16 + globalHist);
			uint32_t h = fsmIndex;
			fsmIndex = m ^ h;
		}
		takenState = globalTable[fsmIndex];
	}
	else {
		DEBUG << "cheking fsmTable in {not globalTable}: fsmTable[" << entryIndex << "][" << fsmIndex << "] = " << fsmTable[entryIndex][fsmIndex] << std::endl;
		takenState = fsmTable[entryIndex][fsmIndex];
	}

	// prepare taken state
	if (fsmState2bool(takenState)) {
		*dst = targetTable[entryIndex];
	}
	else {
		*dst = pc + 4;
	}

	return fsmState2bool(takenState);
}

void BP_update(uint32_t pc, uint32_t targetPc, bool taken, uint32_t pred_dst){
	
	stats.br_num += 1;

	if (!taken) {
		if (pc + 4 != pred_dst)
			stats.flush_num += 1;
	}
	else {
		if (targetPc != pred_dst)
			stats.flush_num += 1;
	}

	uint32_t tag = getRange(pc, 2 + log2(gBtbSize), 1 + log2(gBtbSize) + gTagSize);
	uint32_t entryIndex = getRange(pc, 2, 1 + log2(gBtbSize));// bug fixed 2 -> 1

	DEBUG << "######### update:" << std::endl;
	DEBUG << "pc    = " << std::bitset<32>(pc) << std::endl;
	DEBUG << "tag   = " << std::bitset<32>(tag) << std::endl;
	DEBUG << "entry = " << std::bitset<32>(entryIndex) << std::endl;

	bool isSameTag = tagTable[entryIndex] == tag; // might be redundent

	tagTable[entryIndex] = tag;
	targetTable[entryIndex] = targetPc;
	validBitTable[entryIndex] = true;

	// update history and fsm
	if (!gIsGlobalHist) {
		// history lookup
		uint32_t fsmIndex = histTable[entryIndex];

		// history update
		DEBUG << "history before, taken: " << taken << " " <<  std::bitset<32>(histTable[entryIndex]) << std::endl;
		uint32_t newHistory = histTable[entryIndex];
		DEBUG << "history take,  " << std::bitset<32>(histTable[entryIndex]) << std::endl;
		newHistory = newHistory << 1;
		DEBUG << "history shift, " << std::bitset<32>(newHistory) << std::endl;
		newHistory = newHistory + (int)taken;
		DEBUG << "history add,   " << std::bitset<32>(newHistory) << std::endl;
		uint32_t mask = pow(2, gHistSize) - 1;
		newHistory = newHistory & mask;
		DEBUG << "history mask,  " << std::bitset<32>(newHistory) << std::endl;
		histTable[entryIndex] = newHistory;
		DEBUG << "history after: " << std::bitset<32>(histTable[entryIndex]) << std::endl;

		if (!gIsGlobalTable)
			fsmTable[entryIndex][fsmIndex] = updateFsm(fsmTable[entryIndex][fsmIndex], taken);
		else
			globalTable[fsmIndex] = updateFsm(globalTable[fsmIndex], taken);
	}
	else {
		// fsm update
		uint32_t fsmIndex = globalHist;
		if (gShared == NOT_USING_SHARE)
			fsmIndex = globalHist;
		else if (gShared == USING_SHARE_LSB) {
			uint32_t m = getRange(pc, 2, 2 + gHistSize);
			uint32_t h = fsmIndex;
			fsmIndex = m ^ h;
		}
		else if (gShared == USING_SHARE_MID) {
			uint32_t m = getRange(pc, 16, 16 + globalHist);
			uint32_t h = fsmIndex;
			fsmIndex = m ^ h;
		}

		// history update
		/*uint32_t newHistory = (globalHist << 1 + taken ? 1 : 0);
		uint32_t mask = pow(2, gHistSize) - 1;
		newHistory = newHistory & mask;
		globalHist = newHistory;*/

		DEBUG << "history before, taken: " << taken << " " << std::bitset<32>(histTable[entryIndex]) << std::endl;
		uint32_t newHistory = globalHist;
		DEBUG << "history take,  " << std::bitset<32>(histTable[entryIndex]) << std::endl;
		newHistory = newHistory << 1;
		DEBUG << "history shift, " << std::bitset<32>(newHistory) << std::endl;
		newHistory = newHistory + (int)taken;
		DEBUG << "history add,   " << std::bitset<32>(newHistory) << std::endl;
		uint32_t mask = pow(2, gHistSize) - 1;
		newHistory = newHistory & mask;
		DEBUG << "history mask,  " << std::bitset<32>(newHistory) << std::endl;
		globalHist = newHistory;
		DEBUG << "history after: " << std::bitset<32>(histTable[entryIndex]) << std::endl;
		
		if (!gIsGlobalTable)
			fsmTable[entryIndex][fsmIndex] = updateFsm(fsmTable[entryIndex][fsmIndex], taken);
		else
			globalTable[fsmIndex] = updateFsm(globalTable[fsmIndex], taken);
	}

	printP();
	return;
}

void BP_GetStats(SIM_stats *curStats){
	
	curStats->br_num = stats.br_num;
	curStats->flush_num = stats.flush_num;
	curStats->size = stats.size;

	// deallocate:
	if (!gIsGlobalHist)
		delete[] histTable;

	/// deallocate fsm table:
	if (gIsGlobalTable)
		delete[] globalTable;
	else
	{
		
		for (int i = 0; i < gBtbSize; i++) {
			delete[] fsmTable[i];
		}
		delete[] fsmTable;
	}
	/// deallocate valid bit table:
	delete[] validBitTable;

	/// deallocate tag table and target table
	delete[] tagTable;
	delete[] targetTable;


	return;
}

