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
int debugCount{ 1 };

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
		DEBUG << "range error, (" << start << ", " << end << ")" << std::endl;
		return 0;
	}
	num /= pow(2, start);
	uint32_t mask = pow(2, end + 1 - start) - 1;
	return num & mask;
}

// calcShared: calculate shared xor ONLY IN GLOBAL HISTORY MODE
uint32_t calcShared(uint32_t pc, uint32_t fsmIndex) {
	if (gShared == USING_SHARE_LSB) {
		uint32_t m = getRange(pc, 2, 1 + gHistSize);
		uint32_t h = fsmIndex;
		fsmIndex = m ^ h;
		DEBUG << "[LSB] m:  " << std::bitset<32>(m) << std::endl;
		DEBUG << "[LSB] h:  " << std::bitset<32>(h) << std::endl;
		DEBUG << "[LSB] m^h:" << std::bitset<32>(fsmIndex) << std::endl;
	}
	else if (gShared == USING_SHARE_MID) {
		uint32_t m = getRange(pc, 16, 15 + globalHist);
		uint32_t h = fsmIndex;
		fsmIndex = m ^ h;
		DEBUG << "[MID] m:  " << std::bitset<32>(m) << std::endl;
		DEBUG << "[MID] h:  " << std::bitset<32>(h) << std::endl;
		DEBUG << "[MID] m^h:" << std::bitset<32>(fsmIndex) << std::endl;
	}
	return fsmIndex;
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
	return nextFsm;
}

// printP: print all predictor for debugging purposes. yes, we debug with prints...
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
	
	// initializing global vars
	gBtbSize = btbSize;
	gHistSize = historySize;
	gTagSize = tagSize;
	gFsmState = fsmState;
	gIsGlobalHist = isGlobalHist;
	gIsGlobalTable = isGlobalTable;
	gShared = Shared;

	DEBUG << "init: btbSize: " << gBtbSize << " histSize: " << gHistSize << " tagSize: " << gTagSize << " fsmState: " << gFsmState << " globalHist: " << gIsGlobalHist << " globalTable: " << gIsGlobalTable << " shared: " << gShared << std::endl;

	/// alocate history table:
	if (!gIsGlobalHist) {
		histTable = new uint32_t[gBtbSize];
		for (int i = 0; i < gBtbSize; i++)
			histTable[i] = 0;
	}

	/// allocate fsm table:
	if (gIsGlobalTable) {
        int size = pow(2, gHistSize);
		globalTable = new uint32_t[size];
		for (int i = 0; i < pow(2, gHistSize); i++)
			globalTable[i] = gFsmState;
	}
	else
	{
		fsmTable = new uint32_t * [gBtbSize];
		for (int i = 0; i < gBtbSize; i++) {
            int size = pow(2, gHistSize);
			fsmTable[i] = new uint32_t[size];
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
	for (int i = 0; i < gBtbSize; i++)
		tagTable[i] = 0;
	targetTable = new uint32_t[gBtbSize];
	for (int i = 0; i < gBtbSize; i++)
		targetTable[i] = 0;

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

	return 0;
}

bool BP_predict(uint32_t pc, uint32_t *dst){

	uint32_t tag = getRange(pc, 2 + log2(gBtbSize), 1 + log2(gBtbSize) + gTagSize);
	uint32_t entryIndex = getRange(pc, 2, 1 + log2(gBtbSize));// bug fixed 2 -> 1

	DEBUG << "*************************************line no. " << debugCount++ << std::endl;
	DEBUG << "######### predict:" << std::endl;
	DEBUG << "pc    = " << std::bitset<32>(pc) << std::endl;
	DEBUG << "tag   = " << std::bitset<32>(tag) << std::endl;
	DEBUG << "entry = " << std::bitset<32>(entryIndex) << std::endl;
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
		DEBUG << "globalHist: " << globalHist << std::endl;
	}
	else {
		fsmIndex = histTable[entryIndex];
	}
	DEBUG << "fsmIndex in predict = " << fsmIndex << std::endl;

	// get fsm state
	int takenState;
	if (gIsGlobalTable) {
		fsmIndex = calcShared(pc, fsmIndex);
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
	if (!isSameTag && !gIsGlobalHist) {
		// reset hist reg
		histTable[entryIndex] *= 0;

		DEBUG << "reseting history and fsms" << std::endl;
		// reset fsm states
		if (!gIsGlobalTable) {
			for (int i = 0; i < pow(2, gHistSize); i++) {
				fsmTable[entryIndex][i] = gFsmState;
			}
		}
	}
	else if (!isSameTag && gIsGlobalHist) {
		DEBUG << "reseting fsms" << std::endl;
		// reset fsm states
		if (!gIsGlobalTable) {
			for (int i = 0; i < pow(2, gHistSize); i++) {
				fsmTable[entryIndex][i] = gFsmState;
			}
		}
	}

	tagTable[entryIndex] = tag;
	targetTable[entryIndex] = targetPc;
	validBitTable[entryIndex] = true;

	// update history and fsm
	if (!gIsGlobalHist) {
		// get index from history
		uint32_t fsmIndex = histTable[entryIndex];

		// history update
		uint32_t newHistory = histTable[entryIndex];
		newHistory = newHistory << 1;
		newHistory = newHistory + (int)taken;
		uint32_t mask = pow(2, gHistSize) - 1;
		newHistory = newHistory & mask;
		histTable[entryIndex] = newHistory;

		// fsm update
		if (!gIsGlobalTable)
			fsmTable[entryIndex][fsmIndex] = updateFsm(fsmTable[entryIndex][fsmIndex], taken);
		else {
			fsmIndex = calcShared(pc, fsmIndex);
			globalTable[fsmIndex] = updateFsm(globalTable[fsmIndex], taken);
		}
	}
	else {
		// get index from history
		uint32_t fsmIndex = globalHist;

		// hist update
		uint32_t newHistory = globalHist;
		newHistory = newHistory << 1;
		newHistory = newHistory + (int)taken;
		uint32_t mask = pow(2, gHistSize) - 1;
		newHistory = newHistory & mask;
		globalHist = newHistory;
		
		// fsm update
		if (!gIsGlobalTable)
			fsmTable[entryIndex][fsmIndex] = updateFsm(fsmTable[entryIndex][fsmIndex], taken);
		else {
			fsmIndex = calcShared(pc, fsmIndex);
			globalTable[fsmIndex] = updateFsm(globalTable[fsmIndex], taken);
		}
	}

	if (debug) printP();
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