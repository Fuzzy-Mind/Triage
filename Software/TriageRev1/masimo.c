#include "stm32f0xx.h"
#include "string.h"
#include "masimo.h"

extern UART_HandleTypeDef huart2;
extern MasimoPacket masimoPacket;

uint8_t incommingByte[1];
uint8_t receiveMasimo[64];
int cnt = 0;
int cntOld = 0;
uint8_t lenByte = 0;
uint8_t cSum = 0;
uint8_t cSumOld = 0;
uint8_t result = 0;
uint8_t state = 0;
uint8_t boardStatus = 0;
uint32_t boardSerial = 0;
uint32_t keyCode = 0;
uint16_t spo2 = 0;
uint16_t pulseRate = 0;
uint16_t perfusionIndex = 0;


static const uint32_t manufacturerId = 0x610841E8;
static uint32_t dataCounter = 0;
static uint8_t incomeBuffer[64] = { 0 };
uint8_t outgoingBuffer[32] = { 0 };
static uint8_t dataBuffer[32] = { 0 };

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){	// Masimo Receive Handler
		HAL_UART_Receive_IT(&huart2, incommingByte, 1);
		receiveMasimo[cnt++] = incommingByte[0];
		if(cnt>63){cnt=0;}
		if(incommingByte[0] == 0xAF){
			lenByte = receiveMasimo[1];
			cntOld = cnt;
			if(lenByte+4 == cntOld){	// Paket boyu dogru ise
				for(int j=2; j<(cntOld-2); j++){
					cSum = receiveMasimo[j] + cSum;		// CSUM hesapla
				}
				cSumOld = cSum;
				cSum = 0;
				if(cSumOld == receiveMasimo[cntOld-2]){ // CSUM dogru ise
					switch (receiveMasimo[2]){
						case 0x02 :
							switch (receiveMasimo[cntOld-3]){
								case BAD_PACKET:
									state = 'b';
									break;
								case INVALID_CMD:
									state = 'c';
									break;
								case BANDWIDTH_PROBLEM:
									state = 'w';
									break;
								case INVALID_SYS_APP:
									state = 'a';
									break;
								case BOARD_LOCKED:
									state = BOARD_LOCKED;
									break;
							}
							break;
						case 0x70 :
							boardSerial = receiveMasimo[3];
							boardSerial = (boardSerial<<8) + receiveMasimo[4];
							boardSerial = (boardSerial<<8) + receiveMasimo[5];
							boardSerial = (boardSerial<<8) + receiveMasimo[6];
							state = 'I';
							break;
						case 0x10 :
							state = 'P';
							switch (receiveMasimo[3]){
									case 0x01:
										spo2 = receiveMasimo[4];
										spo2 = (spo2<<8) + receiveMasimo[5];
										break;
									case 0x02:
										pulseRate = receiveMasimo[4];
										pulseRate = (pulseRate<<8) + receiveMasimo[5];
										break;
									case 0x03:
										perfusionIndex = receiveMasimo[4];
										perfusionIndex = (perfusionIndex<<8) + receiveMasimo[5];
										break;
							}
							break;
						case 0x20 :
							state = 'M';
							break;
					}
					result = 1;
				}
				else{
					result = 0;
				}
			}
			cnt=0;
		}
}

void putDataToPacket(const uint8_t data) {
	if (masimoPacket.index >= masimoPacket.length)
		return;

	masimoPacket.data[masimoPacket.index++] = data;
}

void resizePacketData(const uint8_t lenght) {
	masimoPacket.length = lenght;
	masimoPacket.index = 0;

	memset(masimoPacket.data, 0, lenght);
}


void initPacket() {

	masimoPacket.som = 0xA1;
	masimoPacket.eom = 0xAF;
	masimoPacket.index = 0;
	masimoPacket.checksum = 0;
	masimoPacket.length = 0;

	masimoPacket.data = dataBuffer;
	masimoPacket.buffer = outgoingBuffer;

	dataCounter = 0;
	memset(incomeBuffer, 0, 30);
}

void getPacketData() {
	uint8_t checksum;
	uint8_t index;

	masimoPacket.buffer[0] = masimoPacket.som;
	masimoPacket.buffer[1] = masimoPacket.length;

	checksum = 0;
	for (index = 0; index < masimoPacket.length; index++) {
		masimoPacket.buffer[index + (MASIMO_HEADER_SIZE - 2)] = masimoPacket.data[index];
		checksum += masimoPacket.data[index];
	}

	masimoPacket.buffer[index + (MASIMO_HEADER_SIZE - 2)] = checksum;
	masimoPacket.buffer[index + (MASIMO_HEADER_SIZE - 1)] = masimoPacket.eom;
	
	memset(receiveMasimo, 0, sizeof(receiveMasimo));

}

void setBaudrate(baudRate_t baud){
	resizePacketData(2);
	putDataToPacket(0x23);
	putDataToPacket(baud);
	getPacketData();
	HAL_UART_Transmit(&huart2, masimoPacket.buffer, masimoPacket.length+4, 100);
}

void checkStatus(){
	resizePacketData(1);
	putDataToPacket(0x01);
	getPacketData();
	HAL_UART_Transmit(&huart2, masimoPacket.buffer, masimoPacket.length+4, 100);
}

void requestBoardInfo(){
	resizePacketData(1);
	putDataToPacket(0x70);
	getPacketData();
	HAL_UART_Transmit(&huart2, masimoPacket.buffer, masimoPacket.length+4, 100);
}

void unlockBoard(){
	requestBoardInfo();
	HAL_Delay(100);
	keyCode = boardSerial ^ manufacturerId;
	resizePacketData(9);
	putDataToPacket(0x71);
	putDataToPacket(keyCode >> 24);
	putDataToPacket(keyCode >> 16);
	putDataToPacket(keyCode >> 8);
	putDataToPacket(keyCode);
	putDataToPacket(0x00);
	putDataToPacket(0x00);
	putDataToPacket(0x00);
	putDataToPacket(0x5F);
	getPacketData();
	HAL_UART_Transmit(&huart2, masimoPacket.buffer, masimoPacket.length+4, 100);
}

void requestSpo2(){
	resizePacketData(2);
	putDataToPacket(0x10);
	putDataToPacket(0x01);
	getPacketData();
	HAL_UART_Transmit(&huart2, masimoPacket.buffer, masimoPacket.length+4, 100);
}

void requestPulseRate(){
	resizePacketData(2);
	putDataToPacket(0x10);
	putDataToPacket(0x02);
	getPacketData();
	HAL_UART_Transmit(&huart2, masimoPacket.buffer, masimoPacket.length+4, 100);
}

void requestPerfusionIndex(){
	resizePacketData(2);
	putDataToPacket(0x10);
	putDataToPacket(0x03);
	getPacketData();
	HAL_UART_Transmit(&huart2, masimoPacket.buffer, masimoPacket.length+4, 100);
}

void programmingAck(){
	resizePacketData(1);
	putDataToPacket(0x40);
	getPacketData();
	HAL_UART_Transmit(&huart2, masimoPacket.buffer, masimoPacket.length+4, 100);
}