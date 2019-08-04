/*
 * AZctrl_Modbus_leonardo.ino
 * target : ATmega32u4
 * Created: 2/21/2018 4:12:37 PM
 * Author: kanta
 */

#include <Arduino.h>
#include <Ethernet2.h>
#include <EthernetUdp2.h>
#include <SPI.h>
#include <OSCMessage.h>

byte mac[] = { 0x90, 0xA2, 0xDA, 0x10, 0xC8, 0x5F };
IPAddress myIp(10,0,2,101);
IPAddress destIp(10, 0, 2, 10);
unsigned int outPort = 20101;
unsigned int inPort = 20001;
EthernetUDP Udp;

#define TXDEN	3
#define ledPin	13

#define MODBUS_BAUDRATE    57600
uint8_t txBuf[64],txBufLength; // Modbus Tx Buffer
#define TARGET_ALL	0
uint8_t modbus_busy_target = 0;
#define RX_BUFFER_LENGTH	12
enum RxState {STAND_BY =0, SET_REGISTER, GET_DRIVER_STATE, GET_POS};
RxState rxState;
uint32_t lastModbusTxTime;
uint8_t modbusRxLength = 0;

#define DRIVER_CMD_M0	0
#define DRIVER_CMD_M1	1
#define DRIVER_CMD_M2	2
#define DRIVER_CMD_START	3
#define DRIVER_CMD_ZHOME	4
#define DRIVER_CMD_STOP	5
#define DRIVER_CMD_FREE	6
#define DRIVER_CMD_ALM_RST	7
#define DRIVER_CMD_SSTART	11
#define DRIVER_CMD_FW_JOG_P	12
#define DRIVER_CMD_RV_JOG_P	13
#define DRIVER_CMD_FW_POS	14
#define DRIVER_CMD_RV_POS	15

ISR	(USART1_TX_vect) {
	digitalWrite(TXDEN, LOW);
}

// -------------------------------------------
// OSC
// -------------------------------------------

void sendOneData(char* address, int32_t data) {
	OSCMessage newMes(address);
	newMes.add(data);
	Udp.beginPacket(destIp, outPort);
	newMes.send(Udp);
	Udp.endPacket();
	newMes.empty();
}

void sendIdData(char* address, uint8_t id, int32_t data) {
	OSCMessage newMes(address);
	newMes.add(id);
	newMes.add(data);
	Udp.beginPacket(destIp, outPort);
	newMes.send(Udp);
	Udp.endPacket();
	newMes.empty();
}

void setDestIp(OSCMessage &msg ,int addrOffset) {
	destIp = Udp.remoteIP();
	OSCMessage newMes("/newDestIp");
	newMes.add(destIp[3]);
	Udp.beginPacket(destIp, outPort);
	newMes.send(Udp);
	Udp.endPacket();
	newMes.empty();
}

void directDrive(OSCMessage &msg ,int addrOffset) {
	if (modbus_busy_target > 0)
	{
		sendOneData("/modbusBusy", modbus_busy_target);
		return;
	}
	uint8_t target = msg.getInt(0);
	int32_t position = msg.getInt(1);
	uint16_t speed = msg.getInt(2);
	uint16_t acc = msg.getInt(3);
	uint16_t dec = msg.getInt(4);

	directDrive(target,position,speed,acc,dec);
}

void motorStop(OSCMessage &msg ,int addrOffset) {
	if (modbus_busy_target > 0)
	{
		sendOneData("/modbusBusy", modbus_busy_target);
		return;
	}
	uint8_t target = msg.getInt(0);
	setDriverCmd_autoOff(target, DRIVER_CMD_STOP);
}

void getDriverState(OSCMessage &msg ,int addrOffset) {
	if (modbus_busy_target > 0)
	{
		sendOneData("/modbusBusy", modbus_busy_target);
		return;
	}
	uint8_t target = msg.getInt(0);
	if (target == TARGET_ALL)
	{
		sendOneData("/error_target_all", 0);
	} else {
		getDriverState(target);
	}
}

void getPosition(OSCMessage &msg ,int addrOffset) {
	if (modbus_busy_target > 0)
	{
		sendOneData("/modbusBusy", modbus_busy_target);
		return;
	}
	uint8_t target = msg.getInt(0);
	if (target == TARGET_ALL)
	{
		sendOneData("/error_target_all", 0);
		} else {
		getPosition(target);
	}
}

void getModbusBusy(OSCMessage &msg ,int addrOffset) {
	sendOneData("/modbusBusy", modbus_busy_target);
}

void getTxBuf(OSCMessage &msg ,int addrOffset) {

	OSCMessage newMes("/txBuf");
	for (uint8_t i=0; i<txBufLength; i++)
	{
		newMes.add(txBuf[i]);
	}
	Udp.beginPacket(destIp, outPort);
	newMes.send(Udp);
	Udp.endPacket();
	newMes.empty();
}
// -----------------------------------------------------------
// Modbus Tx
// -----------------------------------------------------------

void sendModbusCmd() {
	uint8_t i=0, j=0;
	uint16_t CRCresult = 0xFFFF;
	for(i=0; i<txBufLength; i++) {
		CRCresult ^= txBuf[i];
		for (j=0; j<8; j++) {
			if (CRCresult & 1) {
				CRCresult = (CRCresult >> 1) ^ 0xA001;
				} else {
				CRCresult = CRCresult >> 1;
			}
		}
	}
	txBuf[txBufLength++] = (CRCresult&0xFF);
	txBuf[txBufLength++] = (CRCresult>>8);

	for (i=0; i<txBufLength; i++) {
		Serial.print(txBuf[i]);
		Serial.print(" ");
	}
	Serial.println(" <-- Modbus Tx");
	modbusFlush();
	digitalWrite(TXDEN, HIGH);
	Serial1.write(txBuf, txBufLength);
	modbus_busy_target = txBuf[0];
	if ( txBuf[0] == TARGET_ALL )
	{
		rxState = STAND_BY;
	}
	lastModbusTxTime = millis();
}


void setDriveData(uint8_t target, int32_t position, uint16_t speed, uint16_t acc, uint16_t dec) {
	uint8_t i;
	const uint8_t header[] = {1,16,24,0,0,10,20,0,0,0,1}; // drive data #0, address 0x1800
	for (i=0; i<sizeof(header); i++)
	{
		txBuf[i] = header[i];
	}
	txBuf[0] = target;
	txBufLength = sizeof(header);
	for (i=0; i<4; i++) {
		txBuf[txBufLength++] = (position>>(8*(3-i)))&0xFF;
	}
	for (i=0; i<4; i++) {
		txBuf[txBufLength++] = (speed>>(8*(3-i)))&0xFF;
	}
	for (i=0; i<4; i++) {
		txBuf[txBufLength++] = (acc>>(8*(3-i)))&0xFF;
	}
	for (i=0; i<4; i++) {
		txBuf[txBufLength++] = (dec>>(8*(3-i)))&0xFF;
	}
	rxState = SET_REGISTER;
	modbusRxLength = 8;
	sendModbusCmd();
}

void directDrive(uint8_t target, int32_t position, uint16_t speed, uint16_t acc, uint16_t dec) {
	uint8_t i;
	const uint8_t header[] = {1,16,0,88,0,16,32,0,0,0,255,0,0,0,1};
	const uint8_t footer[] = {0,0,3,232,0,0,0,1};
	for (i=0; i<sizeof(header); i++)
	{
		txBuf[i] = header[i];
	}
	txBuf[0] = target;
	txBufLength = sizeof(header);
	for (i=0; i<4; i++) {
		txBuf[txBufLength++] = (position>>(8*(3-i)))&0xFF;
	}
	for (i=0; i<4; i++) {
		txBuf[txBufLength++] = (speed>>(8*(3-i)))&0xFF;
	}
	for (i=0; i<4; i++) {
		txBuf[txBufLength++] = (acc>>(8*(3-i)))&0xFF;
	}
	for (i=0; i<4; i++) {
		txBuf[txBufLength++] = (dec>>(8*(3-i)))&0xFF;
	}
	for (i=0; i<sizeof(footer); i++) {
		txBuf[txBufLength++] = footer[i];
	}
	rxState = SET_REGISTER;
	modbusRxLength = 8;
	sendModbusCmd();
}

void getDriverState(uint8_t target) {
	uint8_t data[6] = {1,3,0,0x7E,0,2};
	data[0] = target;
	for (uint8_t i=0; i<6; i++)
	{
		txBuf[i] = data[i];
	}
	txBufLength = 6;
	rxState = GET_DRIVER_STATE;
	modbusRxLength = 9;
	sendModbusCmd();
}

void getPosition(uint8_t target) {
	uint8_t data[6] = {1,3,0,0xCC,0,2};
	data[0] = target;
	for (uint8_t i=0; i<6; i++)
	{
		txBuf[i] = data[i];
	}
	txBufLength = 6;
	rxState = GET_POS;
	modbusRxLength = 9;
	sendModbusCmd();
}

void setDriverCmd_autoOff(uint8_t target, uint8_t command) {
	uint8_t i;
	const uint8_t header[] = {1,16,0x00,0x78,0,2,4,0,0};
	for (i=0; i<sizeof(header); i++)
	{
		txBuf[i] = header[i];
	}
	txBufLength = sizeof(header);
	txBuf[0] = target;
	uint16_t t = (1<<command);
	txBuf[txBufLength++] = t>>8;
	txBuf[txBufLength++] = t & 0xFF;
	rxState = SET_REGISTER;
	modbusRxLength = 8;
	sendModbusCmd();
}

// -----------------------------------------------------------
// setup
// -----------------------------------------------------------
void setup()
{
	pinMode(TXDEN, OUTPUT);
	pinMode(ledPin, OUTPUT);
	Serial1.begin(MODBUS_BAUDRATE, SERIAL_8E1);
	Serial.begin(9600);

	UCSR1B |= (1<<TXCIE1);	// TX Complete Interrupt Enable 1

	Ethernet.begin(mac ,myIp);
	Udp.begin(inPort);
}

void OSCMsgReceive() {
	OSCMessage msgIN;
	int size;
	if((size = Udp.parsePacket())>0){
		while(size--)
		msgIN.fill(Udp.read());
		if(!msgIN.hasError()){
			msgIN.route("/directDrive",directDrive);
			msgIN.route("/motorStop",motorStop);
			msgIN.route("/getDriverState",getDriverState);
			msgIN.route("/getPosition",getPosition);
			msgIN.route("/getModbusBusy",getModbusBusy);
			msgIN.route("/setDestIp",setDestIp);
			msgIN.route("/getTxBuf",getTxBuf);
		}
	}
}

void modbusFlush(){
	while(Serial1.available() > 0) {
		char t = Serial1.read();
	}
}

void modbusRx(uint8_t *rxData) {
	uint8_t rxTarget = rxData[0];
	uint16_t state;
	int32_t posi;
	
	switch(rxState) {
		case SET_REGISTER:
			modbus_busy_target = 0;
			break;
		case GET_DRIVER_STATE:
			state = (rxData[5]<<8) | rxData[6];
			sendIdData("/driverState",rxTarget, state);
			modbus_busy_target = 0;
			break;
		case GET_POS:
			posi = ((int32_t)rxData[3]<<24) | ((int32_t)rxData[4]<<16) | (rxData[5]<<8) | rxData[6];
			sendIdData("/position", rxTarget, posi);
			modbus_busy_target = 0;
			break;
		default:
			break;
	}
}

// -----------------------------------------------------------
// loop
// -----------------------------------------------------------
void loop()
{
	OSCMsgReceive();
	byte t = Serial1.available();
	if ( t >= modbusRxLength)
	{
		uint8_t buf[RX_BUFFER_LENGTH];
		for (uint8_t i=0; i<t; i++)
		{
			buf[i] = Serial1.read();
		}
		modbusRx(buf);
	}

	if ( modbus_busy_target > 0)
	{
		if ( (millis()-lastModbusTxTime>=200) || ( lastModbusTxTime > millis() ) )
		{
			modbus_busy_target = 0;
		}
	}

}
