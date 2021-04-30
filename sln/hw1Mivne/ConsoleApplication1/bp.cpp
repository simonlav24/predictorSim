/* 046267 Computer Architecture - Spring 2020 - HW #1 */
/* This file should hold your implementation of the predictor simulator */

#include "bp_api.h"
#include <iostream>

#define VALID_BIT_SIZE 1
#define PC_SIZE 30

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
	if (!gIsGlobalHist)
		histTable = new uint32_t[gBtbSize];

	/// allocate fsm table:
	if (gIsGlobalTable) 
		globalTable = new uint32_t[pow(2, gHistSize)];
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
	 

	return 0;
}

bool BP_predict(uint32_t pc, uint32_t *dst){
	return false;
}

void BP_update(uint32_t pc, uint32_t targetPc, bool taken, uint32_t pred_dst){
	return;
}

void BP_GetStats(SIM_stats *curStats){
	
	curStats->br_num = stats.br_num;
	curStats->flush_num = stats.flush_num;
	curStats->size = stats.size;

	// deallocate:



	return;
}

