#ifndef LIB_MASIMO_MASIMO_H_
#define LIB_MASIMO_MASIMO_H_

#include "stm32f0xx.h"

static const int MASIMO_HEADER_SIZE = 4;
static uint32_t dataCounter;
static uint8_t incomeBuffer[64];
static uint8_t dataBuffer[32];

typedef enum {
	b9600 = 0x00,
	b19200 = 0x01,
	b28800 = 0x02,
	b38400 = 0x03,
	b57600 = 0x04,
} baudRate_t;

typedef enum {
	BAD_PACKET = 0x00,
	INVALID_CMD = 0x01,
	BANDWIDTH_PROBLEM = 0x02,
	INVALID_SYS_APP = 0x03,
	BOARD_LOCKED = 0x04,
	BOARD_SHUTDOWN = 0x09,
} nackStatus;

typedef struct MasimoPacket{
	uint8_t som;
	uint8_t length;
	uint8_t index;
	uint8_t * data;
	uint8_t checksum;
	uint8_t eom;
	uint8_t * buffer;
} MasimoPacket;

void initPacket();
void putDataToPacket(const uint8_t data);
void resizePacketData(const uint8_t lenght);
void getPacketData();
void setBaudrate(baudRate_t baud);
void checkStatus();
void requestBoardInfo();
void unlockBoard();
void programmingAck();
void requestSpo2();
void requestPulseRate();
void requestPerfusionIndex();

#endif
