/***
 * THIS FILE IS AUTOMATICALLY GENERATED
 * Do not edit this file. It will be overwritten when the configure script is run.
 ***/

#ifndef CHAMPSIM_CONSTANTS_H
#define CHAMPSIM_CONSTANTS_H
#include "util.h"
#define BLOCK_SIZE 64ul
#define LOG2_BLOCK_SIZE lg2(BLOCK_SIZE)
#define PAGE_SIZE 4096ul
#define LOG2_PAGE_SIZE lg2(PAGE_SIZE)
#define STAT_PRINTING_PERIOD 10000000ul
#define NUM_CPUS 16ul
#define NUM_CACHES 97u
#define NUM_OPERABLES 130u
#define DRAM_IO_FREQ 2400ul
#define CXL_MODE 0ul
#define DRAM_CHANNELS 2ul
#define DRAM_RANKS 2ul
#define DRAM_BANKS 16ul
#define DRAM_ROWS 131072ul
#define DRAM_COLUMNS 128ul
#define DRAM_CHANNEL_WIDTH 8ul
#define DRAM_WQ_SIZE 64ul
#define DRAM_RQ_SIZE 128ul
#define tRP_DRAM_NANOSECONDS 12.5
#define tRCD_DRAM_NANOSECONDS 12.5
#define tCAS_DRAM_NANOSECONDS 12.5
#define tCXL_DRAM_NANOSECONDS 30
#define DBUS_TURN_AROUND_NANOSECONDS 7.5
#endif
