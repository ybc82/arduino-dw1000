/*
 * Copyright (c) 2015 by Thomas Trojer <thomas@trojer.net> and Leopold Sayous <leosayous@gmail.com>
 * Decawave DW1000 library for arduino.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * @file DW1000SyncRanging.h
 * Arduino global library (source file) working with the DW1000 library 
 * for the Decawave DW1000 UWB transceiver IC.
 *
 * @TODO
 * - remove or debugmode for Serial.print
 * - move strings to flash to reduce ram usage
 * - do not safe duplicate of pin settings
 * - maybe other object structure
 * - use enums instead of preprocessor constants
 *
 * Copyright (c) 2017 by Bocheng Yu <yuuubc@gmail.com>
 * TDOF localization plus synchronization among base node and anchor nodes
 */
#include "constants.h"
#if defined(TDOA)

#include "DW1000SyncRanging.h"
#include "DW1000Device.h"
#include <MatrixMath.h>
#include <BigNumber.h>

DW1000SyncRangingClass DW1000SyncRanging;


//other devices we are going to communicate with which are on our network:
DW1000Device DW1000SyncRangingClass::_networkDevices[MAX_DEVICES];
byte         DW1000SyncRangingClass::_currentAddress[8];
byte         DW1000SyncRangingClass::_currentShortAddress[2];
byte         DW1000SyncRangingClass::_lastSentToShortAddress[2];
volatile 	uint8_t      DW1000SyncRangingClass::_networkDevicesNumber = 0; // TODO short, 8bit?
int16_t      DW1000SyncRangingClass::_lastDistantDevice    = 0; // TODO short, 8bit?
DW1000Mac    DW1000SyncRangingClass::_globalMac;

//module type (anchor or tag)
int16_t      DW1000SyncRangingClass::_type; // TODO enum??

// message flow state
volatile byte    DW1000SyncRangingClass::_expectedMsgId;

// range filter
volatile boolean DW1000SyncRangingClass::_useRangeFilter = false;
uint16_t DW1000SyncRangingClass::_rangeFilterValue = 15;

// message sent/received state
volatile boolean DW1000SyncRangingClass::_sentAck     = false;
volatile boolean DW1000SyncRangingClass::_receivedAck = false;

// protocol error state
boolean          DW1000SyncRangingClass::_protocolFailed = false;

// timestamps to remember
int32_t            DW1000SyncRangingClass::timer           = 0;
int16_t            DW1000SyncRangingClass::counterForBlink = 0; // TODO 8 bit?

// postition output
double 			   DW1000SyncRangingClass::_position[3] = {0, 0, 0};

// data buffer
byte          DW1000SyncRangingClass::data[LEN_DATA];
// reset line to the chip
uint8_t   DW1000SyncRangingClass::_RST;
uint8_t   DW1000SyncRangingClass::_SS;
// watchdog and reset period
uint32_t  DW1000SyncRangingClass::_lastActivity;
uint32_t  DW1000SyncRangingClass::_resetPeriod;
// reply times (same on both sides for symm. ranging)
uint16_t  DW1000SyncRangingClass::_replyDelayTimeUS;
//timer delay
uint16_t  DW1000SyncRangingClass::_timerDelay;
// ranging counter (per second)
uint16_t  DW1000SyncRangingClass::_successRangingCount = 0;
uint32_t  DW1000SyncRangingClass::_rangingCountPeriod  = 0;
//Here our handlers
void (* DW1000SyncRangingClass::_handleNewRange)(void) = 0;
void (* DW1000SyncRangingClass::_handleBlinkDevice)(DW1000Device*) = 0;
void (* DW1000SyncRangingClass::_handleNewDevice)(DW1000Device*) = 0;
void (* DW1000SyncRangingClass::_handleInactiveDevice)(DW1000Device*) = 0;

/* ###########################################################################
 * #### Init and end #######################################################
 * ######################################################################### */

void DW1000SyncRangingClass::initCommunication(uint8_t myRST, uint8_t mySS, uint8_t myIRQ) {
	// reset line to the chip
	_RST              = myRST;
	_SS               = mySS;
	_resetPeriod      = DEFAULT_RESET_PERIOD;
	// reply times (same on both sides for symm. ranging)
	_replyDelayTimeUS = DEFAULT_REPLY_DELAY_TIME;
	//we set our timer delay
	_timerDelay       = DEFAULT_TIMER_DELAY;
	
	
	DW1000.begin(myIRQ, myRST);
	DW1000.select(mySS);
}


void DW1000SyncRangingClass::configureNetwork(uint16_t deviceAddress, uint16_t networkId, const byte mode[]) {
	// general configuration
	DW1000.newConfiguration();
	DW1000.setDefaults();
	DW1000.setDeviceAddress(deviceAddress);
	DW1000.setNetworkId(networkId);
	DW1000.enableMode(mode);
	DW1000.commitConfiguration();
	
}

void DW1000SyncRangingClass::generalStart() {
	// attach callback for (successfully) sent and received messages
	DW1000.attachSentHandler(handleSent);
	DW1000.attachReceivedHandler(handleReceived);
	// anchor starts in receiving mode, awaiting a ranging poll message
	
	
	if(DEBUG) {
		// DEBUG monitoring
		Serial.println("DW1000-arduino");
		// initialize the driver
		
		
		Serial.println("configuration..");
		// DEBUG chip info and registers pretty printed
		char msg[90];
		DW1000.getPrintableDeviceIdentifier(msg);
		Serial.print("Device ID: ");
		Serial.println(msg);
		DW1000.getPrintableExtendedUniqueIdentifier(msg);
		Serial.print("Unique ID: ");
		Serial.print(msg);
		char string[6];
		sprintf(string, "%02X:%02X", _currentShortAddress[0], _currentShortAddress[1]);
		Serial.print(" short: ");
		Serial.println(string);
		
		DW1000.getPrintableNetworkIdAndShortAddress(msg);
		Serial.print("Network ID & Device Address: ");
		Serial.println(msg);
		DW1000.getPrintableDeviceMode(msg);
		Serial.print("Device mode: ");
		Serial.println(msg);
	}
	
	
	// anchor starts in receiving mode, awaiting a ranging poll message
	receiver();
	// for first time ranging frequency computation
	_rangingCountPeriod = millis();

	BigNumber::begin();
}


void DW1000SyncRangingClass::startAsAnchor(char address[], const char shortAddress[], const byte mode[]) {
	//save the address
	DW1000.convertToByte(address, _currentAddress);
	//write the address on the DW1000 chip
	DW1000.setEUI(address);
	Serial.print("device address: ");
	Serial.println(address);
	//we need to define a random short address:
	randomSeed(analogRead(0));
	if(!shortAddress)
	{
		_currentShortAddress[0] = random(0, 256);
		_currentShortAddress[1] = random(0, 256);
	}
	else // short address provided
	{
		_currentShortAddress[0] = shortAddress[0];
		_currentShortAddress[1] = shortAddress[1];
	}

	//we configur the network for mac filtering
	//(device Address, network ID, frequency)
	DW1000SyncRanging.configureNetwork(_currentShortAddress[0]*256+_currentShortAddress[1], 0xDECA, mode);
	
	//general start:
	generalStart();
	
	//defined type as anchor
	_type = ANCHOR;
	
	Serial.println("### ANCHOR ###");
	
}

void DW1000SyncRangingClass::startAsTag(char address[], const char shortAddress[], const byte mode[]) {
	//save the address
	DW1000.convertToByte(address, _currentAddress);
	//write the address on the DW1000 chip
	DW1000.setEUI(address);
	Serial.print("device address: ");
	Serial.println(address);
	//we need to define a random short address:
	randomSeed(analogRead(0));
	if(!shortAddress)
	{
		_currentShortAddress[0] = random(0, 256);
		_currentShortAddress[1] = random(0, 256);
	}
	else // short address provided
	{
		_currentShortAddress[0] = shortAddress[0];
		_currentShortAddress[1] = shortAddress[1];
	}
	
	//we configur the network for mac filtering
	//(device Address, network ID, frequency)
	DW1000SyncRanging.configureNetwork(_currentShortAddress[0]*256+_currentShortAddress[1], 0xDECA, mode);
	
	generalStart();
	//defined type as tag
	_type = TAG;
	
	Serial.println("### TAG ###");
}

void DW1000SyncRangingClass::startAsBase(char address[], const char shortAddress[], const byte mode[]) {
	//save the address
	DW1000.convertToByte(address, _currentAddress);
	//write the address on the DW1000 chip
	DW1000.setEUI(address);
	Serial.print("device address: ");
	Serial.println(address);
	//we need to define a random short address:
	randomSeed(analogRead(0));
	if(!shortAddress)
	{
		_currentShortAddress[0] = random(0, 256);
		_currentShortAddress[1] = random(0, 256);
	}
	else // short address provided
	{
		_currentShortAddress[0] = shortAddress[0];
		_currentShortAddress[1] = shortAddress[1];
	}
	
	//we configur the network for mac filtering
	//(device Address, network ID, frequency)
	DW1000SyncRanging.configureNetwork(_currentShortAddress[0]*256+_currentShortAddress[1], 0xDECA, mode);
	
	generalStart();
	//defined type as base
	_type = BASE;
	
	Serial.println("### BASE ###");
}

boolean DW1000SyncRangingClass::addNetworkDevices(DW1000Device* device, boolean shortAddress) {
	boolean   addDevice = true;
	//we test our network devices array to check
	//we don't already have it
	for(uint8_t i = 0; i < _networkDevicesNumber; i++) {
		if(_networkDevices[i].isAddressEqual(device) && !shortAddress) {
			//the device already exists
			addDevice = false;
			return false;
		}
		else if(_networkDevices[i].isShortAddressEqual(device) && shortAddress) {
			//the device already exists
			addDevice = false;
			return false;
		}
		
	}
	
	if(addDevice) {
		device->setRange(0);
		memcpy(&_networkDevices[_networkDevicesNumber], device, sizeof(DW1000Device));
		_networkDevices[_networkDevicesNumber].setIndex(_networkDevicesNumber);
		_networkDevicesNumber++;
		return true;
	}
	
	return false;
}

boolean DW1000SyncRangingClass::addNetworkDevices(DW1000Device* device) {
	boolean addDevice = true;
	//we test our network devices array to check
	//we don't already have it
	for(uint8_t i = 0; i < _networkDevicesNumber; i++) {
		if(_networkDevices[i].isAddressEqual(device) && _networkDevices[i].isShortAddressEqual(device)) {
			//the device already exists
			addDevice = false;
			return false;
		}
		
	}
	
	if(addDevice) {
		if(_type == ANCHOR) //for now let's start with 1 TAG
		{
			_networkDevicesNumber = 0;
		}
		memcpy(&_networkDevices[_networkDevicesNumber], device, sizeof(DW1000Device));
		_networkDevices[_networkDevicesNumber].setIndex(_networkDevicesNumber);
		_networkDevicesNumber++;
		return true;
	}
	
	return false;
}

void DW1000SyncRangingClass::removeNetworkDevices(int16_t index) {
	//if we have just 1 element
	if(_networkDevicesNumber == 1) {
		_networkDevicesNumber = 0;
	}
	else if(index == _networkDevicesNumber-1) //if we delete the last element
	{
		_networkDevicesNumber--;
	}
	else {
		//we translate all the element wich are after the one we want to delete.
		for(int16_t i = index; i < _networkDevicesNumber-1; i++) { // TODO 8bit?
			memcpy(&_networkDevices[i], &_networkDevices[i+1], sizeof(DW1000Device));
			_networkDevices[i].setIndex(i);
		}
		_networkDevicesNumber--;
	}
}

/* ###########################################################################
 * #### Setters and Getters ##################################################
 * ######################################################################### */

//setters
void DW1000SyncRangingClass::setReplyTime(uint16_t replyDelayTimeUs) { _replyDelayTimeUS = replyDelayTimeUs; }

void DW1000SyncRangingClass::setResetPeriod(uint32_t resetPeriod) { _resetPeriod = resetPeriod; }


DW1000Device* DW1000SyncRangingClass::searchDistantDevice(byte shortAddress[]) {
	//we compare the 2 bytes address with the others
	for(uint16_t i = 0; i < _networkDevicesNumber; i++) { // TODO 8bit?
		if(memcmp(shortAddress, _networkDevices[i].getByteShortAddress(), 2) == 0) {
			//we have found our device !
			return &_networkDevices[i];
		}
	}
	
	return NULL;
}

DW1000Device* DW1000SyncRangingClass::getDistantDevice() {
	//we get the device which correspond to the message which was sent (need to be filtered by MAC address)
	
	return &_networkDevices[_lastDistantDevice];
	
}

DW1000Device* DW1000SyncRangingClass::getBaseDevice()
{
	// get the base module - index 0
	return &_networkDevices[0];
}

DW1000Device* DW1000SyncRangingClass::getAnchorDevice(uint8_t index)
{	// get the anchor device
	return &_networkDevices[index+1];
}

/* ###########################################################################
 * #### Public methods #######################################################
 * ######################################################################### */

void DW1000SyncRangingClass::checkForReset() {
	uint32_t curMillis = millis();
	if(!_sentAck && !_receivedAck) {
		// check if inactive
		if(curMillis-_lastActivity > _resetPeriod) {
			resetInactive();
		}
		return; // TODO cc
	}
}

void DW1000SyncRangingClass::checkForInactiveDevices() {
	for(uint8_t i = 0; i < _networkDevicesNumber; i++) {
		if(_networkDevices[i].isInactive()) {
			if(_handleInactiveDevice != 0) {
				(*_handleInactiveDevice)(&_networkDevices[i]);
			}
			//we need to delete the device from the array:
			removeNetworkDevices(i);
			
		}
	}
}

// TODO check return type
int16_t DW1000SyncRangingClass::detectMessageType(byte datas[]) {
	if(datas[0] == FC_1_BLINK) {
		return BLINK;
	}
	else if(datas[0] == FC_1 && datas[1] == FC_2) {
		//we have a long MAC frame message (ranging init)
		return datas[LONG_MAC_LEN];
	}
	else if(datas[0] == FC_1 && datas[1] == FC_2_SHORT) {
		//we have a short mac frame message (poll, range, range report, etc..)
		return datas[SHORT_MAC_LEN];
	}
}

void DW1000SyncRangingClass::loop() {
	//we check if needed to reset !
	checkForReset();
	uint32_t time = millis(); // TODO other name - too close to "timer"
	if(time-timer > _timerDelay) {
		timer = time;
		timerTick();
	}
	
	if(DEBUG)
	{
		if(_sentAck || _receivedAck) // FOR DEBUGGING
		{
			Serial.print(_sentAck);
			Serial.println(_receivedAck);
		}
	}

	if(_sentAck) { // right after a message is sent
		_sentAck = false;
		// TODO cc
		int messageType = detectMessageType(data);
		// if(DEBUG)
		// {
		// 	Serial.print("S");
		// 	Serial.println(messageType);
		// }
		// if(_type == ANCHOR)
		// 	info1.add('S', messageType);

		// After these commands are sent, the sender should do something. Otherwise, just return.
		if(messageType != POLL_ACK && messageType != POLL)
			return;
		
		//A msg was sent. We launch the ranging protocole when a message was sent
		if(_type == ANCHOR) {
			if(messageType == POLL_ACK) {
				DW1000Device* myDistantDevice = searchDistantDevice(_lastSentToShortAddress);
				
				DW1000.getTransmitTimestamp(myDistantDevice->timePollAckSent);
			}
		}
		else if(_type == BASE) {
			if(messageType == POLL) {
				DW1000Time timePollSent;
				DW1000.getTransmitTimestamp(timePollSent);
				//if the last device we send the POLL is broadcast:
				if(_lastSentToShortAddress[0] == 0xFF && _lastSentToShortAddress[1] == 0xFF) {
					//we save the value for all the devices !
					for(uint16_t i = 0; i < _networkDevicesNumber; i++) {
						_networkDevices[i].timePollSent = timePollSent;
					}
				}
				else {
					//we search the device associated with the last send address
					DW1000Device* myDistantDevice = searchDistantDevice(_lastSentToShortAddress);
					//we save the value just for one device
					myDistantDevice->timePollSent = timePollSent;
				}
			}
			/*
			else if(messageType == RANGE) {
				DW1000Time timeRangeSent;
				DW1000.getTransmitTimestamp(timeRangeSent);
				//if the last device we send the POLL is broadcast:
				if(_lastSentToShortAddress[0] == 0xFF && _lastSentToShortAddress[1] == 0xFF) {
					//we save the value for all the devices ! 
					for(uint16_t i = 0; i < _networkDevicesNumber; i++) {
						_networkDevices[i].timeRangeSent = timeRangeSent;
					}
				}
				else {
					//we search the device associated with the last send address
					DW1000Device* myDistantDevice = searchDistantDevice(_lastSentToShortAddress);
					//we save the value just for one device
					myDistantDevice->timeRangeSent = timeRangeSent;
				}
				
			}
			*/
		}
		
	}
	
	//check for new received message
	if(_receivedAck) {
		_receivedAck = false;
		
		//we read the datas from the modules:
		// get message and parse
		DW1000.getData(data, LEN_DATA);
		// visualizeDatas(data);

		int messageType = detectMessageType(data);

		// if (DEBUG)
		// {
		// 	Serial.println(messageType);
		// }

		// if(_type == ANCHOR)
		// 	info1.add('R', messageType);
		//we have just received a BLINK message from tag
		if(messageType == BLINK && _type == ANCHOR) {
			byte address[8];
			byte shortAddress[2];
			_globalMac.decodeBlinkFrame(data, address, shortAddress);
			//we crate a new device with th tag
			DW1000Device myTag(address, shortAddress);
			
			if(addNetworkDevices(&myTag)) {
				if(_handleBlinkDevice != 0) {
					(*_handleBlinkDevice)(&myTag);
				}
				//we reply by the transmit ranging init message
				transmitRangingInit(&myTag);
				noteActivity();
			}
			_expectedMsgId = POLL;
		}
		else if(messageType == RANGING_INIT && _type == TAG) {
			
			byte address[2];
			_globalMac.decodeLongMACFrame(data, address);
			//we crate a new device with the anchor
			DW1000Device myAnchor(address, true);
			
			if(addNetworkDevices(&myAnchor, true)) {
				if(_handleNewDevice != 0) {
					(*_handleNewDevice)(&myAnchor);
				}
			}
			
			noteActivity();
		}
		else {
			//we have a short mac layer frame !
			byte address[2];
			_globalMac.decodeShortMACFrame(data, address);
			
			
			//we get the device which correspond to the message which was sent (need to be filtered by MAC address)
			DW1000Device* myDistantDevice = searchDistantDevice(address);
			
			if (DEBUG)
			{
				Serial.print("T");
				Serial.print(messageType);
				Serial.print(",E");
				Serial.print(_expectedMsgId);
				Serial.print(",D");
				Serial.println(myDistantDevice->getIndex());
			}

			if((_networkDevicesNumber != 0) && (myDistantDevice == NULL)) {
				// Serial.println("Not found");
				//we don't have the short address of the device in memory
				/*
				Serial.print("unknown: ");
				Serial.print(address[0], HEX);
				Serial.print(":");
				Serial.println(address[1], HEX);
				*/

				return;
			}
			//then we proceed to range protocol
			if(_type == ANCHOR) {
				if(messageType != _expectedMsgId) {
					// unexpected message, start over again (except if already POLL)
					_protocolFailed = true;
				}
				if(messageType == POLL) {
					//we receive a POLL which is a broacast message
					//we need to grab info about it

					int16_t numberDevices = 0;
					memcpy(&numberDevices, data+SHORT_MAC_LEN+1, 1);

					for(uint16_t i = 0; i < numberDevices; i++) {
						//we need to test if this value is for us:
						//we grab the mac address of each devices:
						byte shortAddress[2];
						memcpy(shortAddress, data+SHORT_MAC_LEN+2+i*4, 2);
						
						//we test if the short address is our address
						if(shortAddress[0] == _currentShortAddress[0] && shortAddress[1] == _currentShortAddress[1]) {
							//we grab the replytime wich is for us
							uint16_t replyTime;
							memcpy(&replyTime, data+SHORT_MAC_LEN+2+i*4+2, 2);
							//we configure our replyTime;
							_replyDelayTimeUS = replyTime;
							
							// on POLL we (re-)start, so no protocol failure
							_protocolFailed = false;
							
							DW1000.getReceiveTimestamp(myDistantDevice->timePollReceived);
							//we note activity for our device:
							myDistantDevice->noteActivity();
							//we indicate our next receive message for our ranging protocole
							_expectedMsgId = RANGE;
							transmitPollAck(myDistantDevice);
							noteActivity();
							
							return;
						}
						
					}
					
					
				}
				else if(messageType == RANGE) {
					//we receive a RANGE which is a broacast message
					//we need to grab info about it
					uint8_t numberDevices = 0;
					memcpy(&numberDevices, data+SHORT_MAC_LEN+1, 1);
					
					if(DEBUG)
					{
						Serial.println("Range");
					}
					for(uint8_t i = 0; i < numberDevices; i++) {
						//we need to test if this value is for us:
						//we grab the mac address of each devices:
						byte shortAddress[2];
						memcpy(shortAddress, data+SHORT_MAC_LEN+2+i*17, 2);
						
						//we test if the short address is our address
						if(shortAddress[0] == _currentShortAddress[0] && shortAddress[1] == _currentShortAddress[1]) {
							if(DEBUG)
							{
								Serial.print(i);
								Serial.print(" ");
								Serial.print(shortAddress[0], HEX);
								Serial.println(shortAddress[1], HEX);
							}
							//we grab the replytime wich is for us
							DW1000.getReceiveTimestamp(myDistantDevice->timeRangeReceived);
							// myDistantDevice->timeRangeAllSent = myDistantDevice->timeRangeReceived + DW1000Time(_replyDelayTimeUS, DW1000Time::MICROSECONDS);
							noteActivity();
							_expectedMsgId = POLL;
							
							if(!_protocolFailed) {
								
								myDistantDevice->timePollSent.setTimestamp(data+SHORT_MAC_LEN+4+17*i);
								myDistantDevice->timePollAckReceived.setTimestamp(data+SHORT_MAC_LEN+9+17*i);
								myDistantDevice->timeRangeSent.setTimestamp(data+SHORT_MAC_LEN+14+17*i);
								
								// (re-)compute range as two-way ranging is done
								DW1000Time myTOF;
								computeRangeAsymmetric(myDistantDevice, &myTOF); // CHOSEN RANGING ALGORITHM
								
								float distance = myTOF.getAsMeters();
								
								if (_useRangeFilter) {
									//Skip first range
									if (myDistantDevice->getRange() != 0.0f) {
										distance = filterValue(distance, myDistantDevice->getRange(), _rangeFilterValue);
									}
								}
								
								myDistantDevice->setRXPower(DW1000.getReceivePower());
								myDistantDevice->setRange(distance);
								
								float fp_ampl;
								float pp_index; // peak path index
								myDistantDevice->setFPPower(DW1000.getFirstPathPower(&fp_ampl));
								myDistantDevice->setFPAmpl(fp_ampl);
								myDistantDevice->setPeakAmpl(DW1000.getPeakAmplitude(&pp_index));
								myDistantDevice->setPPIndx(pp_index);
								myDistantDevice->setQuality(DW1000.getReceiveQuality());
								myDistantDevice->setFPIndx(DW1000.getFPIndex());
								
								//we send the RANGE_ALL to all tags
								transmitRangeAll(myDistantDevice);	// specify the distant device
								
								//we have finished our range computation. We send the corresponding handler
								_lastDistantDevice = myDistantDevice->getIndex();
								if(_handleNewRange != 0) {
									(*_handleNewRange)();
								}
								
							}
							else {
								transmitRangeFailed(NULL);
							}
							
							
							return;
						}
						
					}
					
					
				}
			}
			else if(_type == BASE) {
				if(DEBUG)
				{
					Serial.print(myDistantDevice->getShortAddress(), HEX);
					Serial.print(" ");
					Serial.println(myDistantDevice->getIndex());
				}

				// get message and parse
				if(messageType != _expectedMsgId) {
					// unexpected message, start over again
					//not needed ?
					return;
					_expectedMsgId = POLL_ACK;
					return;
				}
				if(messageType == POLL_ACK) {
					DW1000.getReceiveTimestamp(myDistantDevice->timePollAckReceived);
					//we note activity for our device:
					myDistantDevice->noteActivity();
					
					//in the case the message come from our last device:
					if(myDistantDevice->getIndex() == _networkDevicesNumber-1) {
						_expectedMsgId = RANGE_ALL;
						//and transmit the next message (range) of the ranging protocole (in broadcast)
						transmitRange(NULL);
					}
				}
				else if(messageType == RANGE_ALL) {
					//we note activity for our device:
					myDistantDevice->noteActivity();

					//in the case the message come from our last device:
					if(myDistantDevice->getIndex() == _networkDevicesNumber-1)
						_expectedMsgId = POLL_ACK;

					return;

					/*
					float curRange;
					memcpy(&curRange, data+1+SHORT_MAC_LEN, 4);
					float curRXPower;
					memcpy(&curRXPower, data+5+SHORT_MAC_LEN, 4);
					float curFPPower;
					memcpy(&curFPPower, data+9+SHORT_MAC_LEN, 4);
					float curPkAmpl;
					memcpy(&curPkAmpl, data+13+SHORT_MAC_LEN, 4);
					float curFPAmpl;
					memcpy(&curFPAmpl, data+17+SHORT_MAC_LEN, 4);
					float curPPIndx;
					memcpy(&curPPIndx, data+21+SHORT_MAC_LEN, 4);
					float curFPIndx;
					memcpy(&curFPIndx, data+25+SHORT_MAC_LEN, 4);

					if (_useRangeFilter) {
						//Skip first range
						if (myDistantDevice->getRange() != 0.0f) {
							curRange = filterValue(curRange, myDistantDevice->getRange(), _rangeFilterValue);
						}
					}

					//we have a new range to save !
					myDistantDevice->setRange(curRange);
					myDistantDevice->setRXPower(curRXPower);
					myDistantDevice->setFPPower(curFPPower);
					myDistantDevice->setPeakAmpl(curPkAmpl);
					myDistantDevice->setFPAmpl(curFPAmpl);
					myDistantDevice->setPPIndx(curPPIndx);
					myDistantDevice->setFPIndx(curFPIndx);
					
					//We can call our handler !
					//we have finished our range computation. We send the corresponding handler
					_lastDistantDevice = myDistantDevice->getIndex();
					if(_handleNewRange != 0) {
						(*_handleNewRange)();
					}
					*/
				}
				else if(messageType == RANGE_FAILED) {
					//not needed as we have a timer;
					_expectedMsgId = POLL_ACK;
					return;
				}
			}
			else if(_type == TAG)
			{
				if(DEBUG)
				{
					Serial.print('R');
					Serial.println(messageType);
				}
		 		// get message and parse
				if(messageType != _expectedMsgId) {
					// unexpected message, start over again
					//not needed ?
					return;
					_expectedMsgId = POLL_ACK;
					return;
				}

				if(messageType == POLL) {
					//we receive a POLL which is a broacast message from BASE
					//we need to grab info about it: POLL_SentTime

					int16_t numberDevices = 0;
					memcpy(&numberDevices, data+SHORT_MAC_LEN+1, 1);


					// on POLL we (re-)start, so no protocol failure
					_protocolFailed = false;
					
					DW1000.getReceiveTimestamp(myDistantDevice->timePollReceived);
					//we note activity for our device:
					myDistantDevice->noteActivity();
					//we indicate our next receive message for our ranging protocole
					_expectedMsgId = RANGE;
					noteActivity();
					
					return;
				}
				else if(messageType == RANGE) {
					//we receive a RANGE which is a broacast message from BASE
					//we need to grab info about it
					//we grab the replytime wich is for us
					DW1000.getReceiveTimestamp(myDistantDevice->timeRangeReceived);
					// and some other timestamps provided by BASE
					myDistantDevice->timePollSent.setTimestamp(data+SHORT_MAC_LEN+4);
					myDistantDevice->timeRangeSent.setTimestamp(data+SHORT_MAC_LEN+14);
					noteActivity();
					_expectedMsgId = RANGE_ALL;
					
					return;
				}
				else if(messageType == RANGE_ALL) {
					// RANGE_ALL from ANCHORs
					//we grab the replytime wich is for us
					DW1000.getReceiveTimestamp(myDistantDevice->timeRangeAllReceived);

					static uint8_t numActiveAnchors = 0;
					numActiveAnchors++;

					float curRange;
					memcpy(&curRange, data+1+SHORT_MAC_LEN, 4);
					float curRXPower;
					memcpy(&curRXPower, data+5+SHORT_MAC_LEN, 4);
					float curFPPower;
					memcpy(&curFPPower, data+9+SHORT_MAC_LEN, 4);
					float curPkAmpl;
					memcpy(&curPkAmpl, data+13+SHORT_MAC_LEN, 4);
					float curFPAmpl;
					memcpy(&curFPAmpl, data+17+SHORT_MAC_LEN, 4);
					float curPPIndx;
					memcpy(&curPPIndx, data+21+SHORT_MAC_LEN, 4);
					float curFPIndx;
					memcpy(&curFPIndx, data+25+SHORT_MAC_LEN, 4);

					if (_useRangeFilter) {
						//Skip first range
						if (myDistantDevice->getRange() != 0.0f) {
							curRange = filterValue(curRange, myDistantDevice->getRange(), _rangeFilterValue);
						}
					}

					//we have a new range to save !
					myDistantDevice->setRange(curRange);
					myDistantDevice->setRXPower(curRXPower);
					myDistantDevice->setFPPower(curFPPower);
					myDistantDevice->setPeakAmpl(curPkAmpl);
					myDistantDevice->setFPAmpl(curFPAmpl);
					myDistantDevice->setPPIndx(curPPIndx);
					myDistantDevice->setFPIndx(curFPIndx);
					myDistantDevice->timePollReceived.setTimestamp(data+29+SHORT_MAC_LEN);
					myDistantDevice->timeRangeReceived.setTimestamp(data+34+SHORT_MAC_LEN);
					myDistantDevice->timeRangeAllSent.setTimestamp(data+39+SHORT_MAC_LEN);

					noteActivity();
					//We can call our handler !
					//we have finished our range computation. We send the corresponding handler
					if(myDistantDevice->getIndex() == _networkDevicesNumber-1) { // This
						_expectedMsgId = POLL;
					}	
					_lastDistantDevice = myDistantDevice->getIndex();

					if(_handleNewRange != 0) {
						(*_handleNewRange)();
					}

					if(myDistantDevice->getIndex() == _networkDevicesNumber-1) { // This
						// TDOF information
						// if(_handleNewRange != 0) {
						// 	(*_handleNewRange)();
						// }

						// TDOA time calculation
						double distanceRel[4];
						if (numActiveAnchors == _networkDevicesNumber - 1)
						{	// enough information
							computeRangeTDOA(NULL, distanceRel);
							uint8_t i = 0;
							uint8_t cnt_positive_relative_distance = 0;
							for (i = 0; i < _networkDevicesNumber-1; i++)
							{
								if (fabs(distanceRel[i]) > 50)
									break;
								if (distanceRel[i] < -10)
								 	break;
							}

							if (i == _networkDevicesNumber-1)
							{ 	// good distance measurement
								for(uint8_t i = 0; i < _networkDevicesNumber-1; i++)
								{
									Serial.print(distanceRel[i]);
									Serial.print('\t');
								}
								computePositionTDOA(distanceRel, _position);

								static int t_now = 0;
								static int t_last = 0;
								Serial.print("P");			Serial.print('\t');
								Serial.print(_position[0]);	Serial.print('\t');
								Serial.print(_position[1]); Serial.print('\t');
								t_now = millis();
								Serial.println(t_now - t_last);
								t_last = t_now;
							}
						}	
						// computeRangeAsymmetric(myDistantDevice, &myTOF); // CHOSEN RANGING ALGORITHM
						// float distance = myTOF.getAsMeters();

						numActiveAnchors=0;
					}	

				}
			}
		}
	}
}

void DW1000SyncRangingClass::useRangeFilter(boolean enabled) {
	_useRangeFilter = enabled;
}

void DW1000SyncRangingClass::setRangeFilterValue(uint16_t newValue) {
	if (newValue < 2) {
		_rangeFilterValue = 2;
	}else{
		_rangeFilterValue = newValue;
	}
}


/* ###########################################################################
 * #### Private methods and Handlers for transmit & Receive reply ############
 * ######################################################################### */


void DW1000SyncRangingClass::handleSent() {
	// status change on sent success
	_sentAck = true;
}

void DW1000SyncRangingClass::handleReceived() {
	// status change on received success
	_receivedAck = true;
}


void DW1000SyncRangingClass::noteActivity() {
	// update activity timestamp, so that we do not reach "resetPeriod"
	_lastActivity = millis();
}

void DW1000SyncRangingClass::resetInactive() {
	//if inactive
	if(_type == ANCHOR) {
		_expectedMsgId = POLL;
		receiver();
	}
	noteActivity();
}

void DW1000SyncRangingClass::timerTick() {
	if(_networkDevicesNumber > 0){ // && counterForBlink != 0) {
		// if(_type == TAG) {
		// 	_expectedMsgId = POLL;
		// }
		
		if(_type == BASE) {
			_expectedMsgId = POLL_ACK;
			//send a prodcast poll
			transmitPoll(NULL);
		}
	}
	else if(counterForBlink == 0) {
		if(_type == TAG) {
			// transmitBlink();
		}
		//check for inactive devices if we are a TAG or ANCHOR
		// checkForInactiveDevices();
	}
	counterForBlink++;
	if(counterForBlink > 20) {
		counterForBlink = 0;
	}
}


void DW1000SyncRangingClass::copyShortAddress(byte address1[], byte address2[]) {
	*address1     = *address2;
	*(address1+1) = *(address2+1);
}

/* ###########################################################################
 * #### Methods for ranging protocole   ######################################
 * ######################################################################### */

void DW1000SyncRangingClass::transmitInit() {
	DW1000.newTransmit();
	DW1000.setDefaults();
}


void DW1000SyncRangingClass::transmit(byte datas[]) {
	DW1000.setData(datas, LEN_DATA);
	DW1000.startTransmit();
}


void DW1000SyncRangingClass::transmit(byte datas[], DW1000Time time) {
	DW1000.setDelay(time);
	DW1000.setData(data, LEN_DATA);
	DW1000.startTransmit();
}

void DW1000SyncRangingClass::transmitAbs(byte datas[], DW1000Time time) {
	/* transmit packet at absolute time stamp */
	DW1000.setDelayAbs(time);
	DW1000.setData(data, LEN_DATA);
	DW1000.startTransmit();
}

void DW1000SyncRangingClass::transmitBlink() {
	transmitInit();
	_globalMac.generateBlinkFrame(data, _currentAddress, _currentShortAddress);
	transmit(data);
}

void DW1000SyncRangingClass::transmitRangingInit(DW1000Device* myDistantDevice) {
	transmitInit();
	//we generate the mac frame for a ranging init message
	_globalMac.generateLongMACFrame(data, _currentShortAddress, myDistantDevice->getByteAddress());
	//we define the function code
	data[LONG_MAC_LEN] = RANGING_INIT;
	
	copyShortAddress(_lastSentToShortAddress, myDistantDevice->getByteShortAddress());
	
	transmit(data);
}

void DW1000SyncRangingClass::transmitPoll(DW1000Device* myDistantDevice) {
	
	transmitInit();
	
	if(myDistantDevice == NULL) {
		//we need to set our timerDelay:
		_timerDelay = DEFAULT_TIMER_DELAY+(uint16_t)(_networkDevicesNumber*3*DEFAULT_REPLY_DELAY_TIME/1000); // YBC: was 3 not 4

		byte shortBroadcast[2] = {0xFF, 0xFF};
		_globalMac.generateShortMACFrame(data, _currentShortAddress, shortBroadcast);
		data[SHORT_MAC_LEN]   = POLL;
		//we enter the number of devices
		data[SHORT_MAC_LEN+1] = _networkDevicesNumber;
		
		for(uint8_t i = 0; i < _networkDevicesNumber; i++) {
			//each devices have a different reply delay time.
			_networkDevices[i].setReplyTime((2*i+1)*DEFAULT_REPLY_DELAY_TIME);
			//we write the short address of our device:
			memcpy(data+SHORT_MAC_LEN+2+4*i, _networkDevices[i].getByteShortAddress(), 2);
			
			//we add the replyTime
			uint16_t replyTime = _networkDevices[i].getReplyTime();
			memcpy(data+SHORT_MAC_LEN+2+2+4*i, &replyTime, 2);

		}
		
		copyShortAddress(_lastSentToShortAddress, shortBroadcast);
		
	}
	else {
		//we redefine our default_timer_delay for just 1 device;
		_timerDelay = DEFAULT_TIMER_DELAY;
		
		_globalMac.generateShortMACFrame(data, _currentShortAddress, myDistantDevice->getByteShortAddress());
		
		data[SHORT_MAC_LEN]   = POLL;
		data[SHORT_MAC_LEN+1] = 1;
		uint16_t replyTime = myDistantDevice->getReplyTime();
		memcpy(data+SHORT_MAC_LEN+2, &replyTime, sizeof(uint16_t)); // todo is code correct?
		
		copyShortAddress(_lastSentToShortAddress, myDistantDevice->getByteShortAddress());
	}
	
	transmit(data);
}


void DW1000SyncRangingClass::transmitPollAck(DW1000Device* myDistantDevice) {
	transmitInit();
	_globalMac.generateShortMACFrame(data, _currentShortAddress, myDistantDevice->getByteShortAddress());
	data[SHORT_MAC_LEN] = POLL_ACK;
	// delay the same amount as ranging tag
	DW1000Time deltaTime = DW1000Time(_replyDelayTimeUS, DW1000Time::MICROSECONDS);
	copyShortAddress(_lastSentToShortAddress, myDistantDevice->getByteShortAddress());
	transmit(data, deltaTime);
}


void DW1000SyncRangingClass::transmitRange(DW1000Device* myDistantDevice) {
	//transmit range need to accept broadcast for multiple anchor
	transmitInit();
	
	
	if(myDistantDevice == NULL) {
		//we need to set our timerDelay:
		_timerDelay = DEFAULT_TIMER_DELAY+(uint16_t)(_networkDevicesNumber*3*DEFAULT_REPLY_DELAY_TIME/1000);
		
		byte shortBroadcast[2] = {0xFF, 0xFF};
		_globalMac.generateShortMACFrame(data, _currentShortAddress, shortBroadcast);
		data[SHORT_MAC_LEN]   = RANGE;
		//we enter the number of devices
		data[SHORT_MAC_LEN+1] = _networkDevicesNumber;
		
		// delay sending the message and remember expected future sent timestamp
		DW1000Time deltaTime     = DW1000Time(DEFAULT_REPLY_DELAY_TIME, DW1000Time::MICROSECONDS);
		DW1000Time timeRangeSent = DW1000.setDelay(deltaTime);
		
		for(uint8_t i = 0; i < _networkDevicesNumber; i++) {
			//we write the short address of our device:
			memcpy(data+SHORT_MAC_LEN+2+17*i, _networkDevices[i].getByteShortAddress(), 2);
			
			
			//we get the device which correspond to the message which was sent (need to be filtered by MAC address)
			_networkDevices[i].timeRangeSent = timeRangeSent;
			_networkDevices[i].timePollSent.getTimestamp(data+SHORT_MAC_LEN+4+17*i);
			_networkDevices[i].timePollAckReceived.getTimestamp(data+SHORT_MAC_LEN+9+17*i);
			_networkDevices[i].timeRangeSent.getTimestamp(data+SHORT_MAC_LEN+14+17*i);
			
		}
		
		copyShortAddress(_lastSentToShortAddress, shortBroadcast);
		
	}
	else {
		_globalMac.generateShortMACFrame(data, _currentShortAddress, myDistantDevice->getByteShortAddress());
		data[SHORT_MAC_LEN] = RANGE;
		// delay sending the message and remember expected future sent timestamp
		DW1000Time deltaTime = DW1000Time(_replyDelayTimeUS, DW1000Time::MICROSECONDS);
		//we get the device which correspond to the message which was sent (need to be filtered by MAC address)
		myDistantDevice->timeRangeSent = DW1000.setDelay(deltaTime);
		myDistantDevice->timePollSent.getTimestamp(data+1+SHORT_MAC_LEN);
		myDistantDevice->timePollAckReceived.getTimestamp(data+6+SHORT_MAC_LEN);
		myDistantDevice->timeRangeSent.getTimestamp(data+11+SHORT_MAC_LEN);
		copyShortAddress(_lastSentToShortAddress, myDistantDevice->getByteShortAddress());
	}
	
	
	transmit(data);
}

void DW1000SyncRangingClass::transmitRangeAll(DW1000Device* myDistantDevice) {
	transmitInit();

	byte shortBroadcast[2] = {0xFF, 0xFF}; // must broadcast
	_globalMac.generateShortMACFrame(data, _currentShortAddress, shortBroadcast);
	data[SHORT_MAC_LEN] = RANGE_ALL;
	// write final ranging result
	float curRange   = myDistantDevice->getRange();
	float curRXPower = myDistantDevice->getRXPower();
	float curFPPower = myDistantDevice->getFPPower();
	float curPkAmpl	 = myDistantDevice->getPeakAmpl();
	float curFPAmpl = myDistantDevice->getFPAmpl();
	float curPPIndx = myDistantDevice->getPPIndx();
	float curFPIndx = myDistantDevice->getFPIndx();
	//We add the Range and then the RXPower
	memcpy(data+1+SHORT_MAC_LEN, &curRange, 4);
	memcpy(data+5+SHORT_MAC_LEN, &curRXPower, 4);
	memcpy(data+9+SHORT_MAC_LEN, &curFPPower, 4);
	memcpy(data+13+SHORT_MAC_LEN, &curPkAmpl, 4);
	memcpy(data+17+SHORT_MAC_LEN, &curFPAmpl, 4);
	memcpy(data+21+SHORT_MAC_LEN, &curPPIndx, 4);
	memcpy(data+25+SHORT_MAC_LEN, &curFPIndx, 4);
	// delay sending the message and remember expected future sent timestamp
	DW1000Time deltaTime     = DW1000Time(_replyDelayTimeUS, DW1000Time::MICROSECONDS);
	DW1000Time timeRangeAllSent = DW1000.setDelay(deltaTime);
	myDistantDevice->timeRangeAllSent = timeRangeAllSent;
	// add timestamps for tags
	myDistantDevice->timePollReceived.getTimestamp(data+29+SHORT_MAC_LEN);
	myDistantDevice->timeRangeReceived.getTimestamp(data+34+SHORT_MAC_LEN);
	myDistantDevice->timeRangeAllSent.getTimestamp(data+39+SHORT_MAC_LEN);

	// delayed the same amount as ranging tag
	copyShortAddress(_lastSentToShortAddress, myDistantDevice->getByteShortAddress());
	transmit(data);

	// add time delay - in an abstract way
	//copyShortAddress(_lastSentToShortAddress, myDistantDevice->getByteShortAddress());
	//transmitAbs(data, DW1000Time(_replyDelayTimeUS, DW1000Time::MICROSECONDS) + myDistantDevice->timeRangeReceived); // YBC: Is this correct?
}

void DW1000SyncRangingClass::transmitRangeFailed(DW1000Device* myDistantDevice) {
	transmitInit();
	_globalMac.generateShortMACFrame(data, _currentShortAddress, myDistantDevice->getByteShortAddress());
	data[SHORT_MAC_LEN] = RANGE_FAILED;
	
	copyShortAddress(_lastSentToShortAddress, myDistantDevice->getByteShortAddress());
	transmit(data);
}

void DW1000SyncRangingClass::receiver() {
	DW1000.newReceive();
	DW1000.setDefaults();
	// so we don't need to restart the receiver manually
	DW1000.receivePermanently(true);
	DW1000.startReceive();
}


/* ###########################################################################
 * #### Methods for range computation and corrections  #######################
 * ######################################################################### */


void DW1000SyncRangingClass::computeRangeAsymmetric(DW1000Device* myDistantDevice, DW1000Time* myTOF) {
	// asymmetric two-way ranging (more computation intense, less error prone)
	DW1000Time round1 = (myDistantDevice->timePollAckReceived-myDistantDevice->timePollSent).wrap();
	DW1000Time reply1 = (myDistantDevice->timePollAckSent-myDistantDevice->timePollReceived).wrap();
	DW1000Time round2 = (myDistantDevice->timeRangeReceived-myDistantDevice->timePollAckSent).wrap();
	DW1000Time reply2 = (myDistantDevice->timeRangeSent-myDistantDevice->timePollAckReceived).wrap();
	
	myTOF->setTimestamp((round1*round2-reply1*reply2)/(round1+round2+reply1+reply2));
	/*
	Serial.print("timePollAckReceived ");myDistantDevice->timePollAckReceived.print();
	Serial.print("timePollSent ");myDistantDevice->timePollSent.print();
	Serial.print("round1 "); Serial.println((long)round1.getTimestamp());
	
	Serial.print("timePollAckSent ");myDistantDevice->timePollAckSent.print();
	Serial.print("timePollReceived ");myDistantDevice->timePollReceived.print();
	Serial.print("reply1 "); Serial.println((long)reply1.getTimestamp());
	
	Serial.print("timeRangeReceived ");myDistantDevice->timeRangeReceived.print();
	Serial.print("timePollAckSent ");myDistantDevice->timePollAckSent.print();
	Serial.print("round2 "); Serial.println((long)round2.getTimestamp());
	
	Serial.print("timeRangeSent ");myDistantDevice->timeRangeSent.print();
	Serial.print("timePollAckReceived ");myDistantDevice->timePollAckReceived.print();
	Serial.print("reply2 "); Serial.println((long)reply2.getTimestamp());
	 */
}

void DW1000SyncRangingClass::computeRangeTDOA(DW1000Device* myDistantDevice, double* distanceRel) {
	// under construction
	DW1000Device* base = getBaseDevice();
	int nAnchors = _networkDevicesNumber - 1;
	DW1000Device* anchors[MAX_DEVICES];
	for (uint8_t i = 0; i < nAnchors; i++)
		anchors[i] = getAnchorDevice(i);
	
	DW1000Time baseEcho = (base->timeRangeSent - base->timePollSent).wrap();
	DW1000Time tagEcho = (base->timeRangeReceived - base->timePollReceived).wrap();
	DW1000Time anchorsEcho[MAX_DEVICES];
	DW1000Time tagDelay[MAX_DEVICES];
	DW1000Time anchorDelay[MAX_DEVICES];
	for (uint8_t i = 0; i < nAnchors; i++)
	{
		anchorsEcho[i] = (anchors[i]->timeRangeReceived - anchors[i]->timePollReceived).wrap_0();
		tagDelay[i] = (anchors[i]->timeRangeAllReceived - base->timeRangeReceived).wrap_0();
		anchorDelay[i] = (anchors[i]->timeRangeAllSent - anchors[i]->timeRangeReceived).wrap_0();
		// Serial.print(anchors[i]->timeRangeAllReceived); 	Serial.print('\t');
		// Serial.print(anchorsEcho[i]);	Serial.print('\t');
		// Serial.print(tagEcho);			Serial.print('\t');
		// Serial.print(anchorDelay[i]);	Serial.print('\t');
		// Serial.print(tagDelay[i]); 		Serial.print('\t');
		// Serial.print(anchors[i]->getRange());			Serial.print('\t');
		// Serial.print(base->timeRangeReceived);		Serial.print('\t');
		// 		DW1000Time tTemp = anchorDelay[i];
		// tTemp *= tagEcho;
		// tTemp /= anchorsEcho[i];
		// Serial.print((tTemp).wrap()); 
		// Serial.println();

	}
		
	BigNumber t_temp;
	for (uint8_t i = 0; i < nAnchors; i++)
	{
		// - Solution I: 2-bit accuracy loss
		// int64_t td = (tagDelay[i] - anchorDelay[i]*(tagEcho/DW1000Time((int64_t)(4)))/anchorsEcho[i]*DW1000Time((int64_t)(4))).wrap_0().getTimestamp();  
		// distanceRel[i] = (long long)td * DW1000Time::DISTANCE_OF_RADIO - anchors[i]->getRange();

		// - Solution II: no accuracy loss
		BigNumber td_big = BigNumber(anchorDelay[i].getTimestamp());
		td_big *= tagEcho.getTimestamp();
		td_big /= anchorsEcho[i].getTimestamp();
		t_temp = td_big;
		td_big = BigNumber(tagDelay[i].getTimestamp()) - td_big;
		distanceRel[i] = (long long)td_big * DW1000Time::DISTANCE_OF_RADIO - anchors[i]->getRange();

		// Serial.print(distanceRel[i]-distanceRel[0]);
		// Serial.print('\t');
	}
	// Serial.println();

}

/*
	TDOA algorithm
	Input: relative distance (anchors to tag) [m]
	Output: position of tag {x, y, z} [m]
*/
#define N_ANCHORS 	5 	// number of anchors
#define N_ANCHORS_COMBINATION	10	// number of possible combination of anchors
#define DIMENSION 	2	// 2D or 3D
void DW1000SyncRangingClass::computePositionTDOA(double* distanceRel, double* pos)
{
	// const double posAnchors[N_ANCHORS][DIMENSION] = {{1.8288, 3.7846}, {2.413, 1.27}, {0.5842, 2.5146}, {0.3556, 0.56388}};	// {x, y, z} [m]
	// const double posAnchors[N_ANCHORS][DIMENSION] = {{2.7686,2.8575}, {2.7686,0.6477}, {0.0508,0.7747}, {0.0508,2.8194}, {1.5494, 4.4196}};	// {x, y, z} [m] // office setup
	const double posAnchors[N_ANCHORS][DIMENSION] = {{1.0668, -6.6548}, {17.2212, -2.4638}, {-0.2032, 7.874}, {-0.1778, 0.5842}, {13.1826, 8.5217}};

	// Solution I
	/*
	double b[N_ANCHORS];
	double RmSquare[N_ANCHORS]; 	// R-measured square
	uint8_t i, j;
	
	double matA[N_ANCHORS-1][DIMENSION+1];
	double matAt[DIMENSION+1][N_ANCHORS-1];
	double matAAt_inv[DIMENSION+1][DIMENSION+1];
	double matB[N_ANCHORS-1];
	double matAtB[DIMENSION+1];
	for (i = 0; i < N_ANCHORS; i++)
	{
		b[i] = 0;
		for (j = 0; j < DIMENSION; j++)
			b[i] += posAnchors[i][j] * posAnchors[i][j];

		RmSquare[i] = distanceRel[i] * distanceRel[i];
	}

	for (i = 0; i < N_ANCHORS-1; i++)
	{
		for (j = 0; j < DIMENSION; j++)
		{
			matA[i][j] = -2*posAnchors[i+1][j] + 2*posAnchors[0][j];
		}
		matA[i][DIMENSION] = 2*(distanceRel[i+1] - distanceRel[0]);
		matB[i] = (b[0] - b[i+1]) + (RmSquare[i+1] - RmSquare[0]);
	}

	// This part could be A^-1*B 
	// Matrix.Invert(*matA, 3);	// invA
	// Matrix.Multiply(*matA, matB, N_ANCHORS-1, DIMENSION+1, 1, pos);
	// Or Least Square
	Matrix.Transpose(*matA, N_ANCHORS-1, DIMENSION+1, *matAt);
	Matrix.Multiply(*matAt, *matA, DIMENSION+1, N_ANCHORS-1, DIMENSION+1, *matAAt_inv);
	Matrix.Invert(*matAAt_inv, DIMENSION+1);
	Matrix.Multiply(*matAt, matB, DIMENSION+1, N_ANCHORS-1, 1, matAtB);
	Matrix.Multiply(*matAAt_inv, matAtB, DIMENSION+1, DIMENSION+1, 1, pos);
	*/

	// Solution I + combination of anchors
	///*
	double b[N_ANCHORS];			// b_i = A_ix^2 + A_iy^2
	double RmSquare[N_ANCHORS]; 	// R-measured square
	uint8_t i, j, i_rel, index;
	
	double matA[N_ANCHORS_COMBINATION][DIMENSION+1];
	double matAt[DIMENSION+1][N_ANCHORS_COMBINATION];
	double matAAt_inv[DIMENSION+1][DIMENSION+1];
	double matB[N_ANCHORS_COMBINATION];
	double matAtB[DIMENSION+1];
	for (i = 0; i < N_ANCHORS; i++)
	{
		b[i] = 0;
		for (j = 0; j < DIMENSION; j++)
			b[i] += posAnchors[i][j] * posAnchors[i][j];

		RmSquare[i] = distanceRel[i] * distanceRel[i];
	}

	index = 0;
	for (i = 1; i < N_ANCHORS; i++)
	{
		for (i_rel = 0; i_rel < i; i_rel++)
		{
			for (j = 0; j < DIMENSION; j++)
			{
				matA[index][j] = -2*posAnchors[i][j] + 2*posAnchors[i_rel][j];
			}
			matA[index][DIMENSION] = 2*(distanceRel[i] - distanceRel[i_rel]);
			matB[index] = (b[i_rel] - b[i]) + (RmSquare[i] - RmSquare[i_rel]);

			index++;
		}
	}

	// This part could be A^-1*B 
	// Matrix.Invert(*matA, 3);	// invA
	// Matrix.Multiply(*matA, matB, N_ANCHORS-1, DIMENSION+1, 1, pos);
	// Or Least Square
	Matrix.Transpose(*matA, N_ANCHORS_COMBINATION, DIMENSION+1, *matAt);
	Matrix.Multiply(*matAt, *matA, DIMENSION+1, N_ANCHORS_COMBINATION, DIMENSION+1, *matAAt_inv);
	Matrix.Invert(*matAAt_inv, DIMENSION+1);
	Matrix.Multiply(*matAt, matB, DIMENSION+1, N_ANCHORS_COMBINATION, 1, matAtB);
	Matrix.Multiply(*matAAt_inv, matAtB, DIMENSION+1, DIMENSION+1, 1, pos);
	// */

	// Solution II
	/*
	double matA[N_ANCHORS-2][DIMENSION];
	double matB[N_ANCHORS-2];
	for (i = 2; i < N_ANCHORS; i++)	// i: index of anchors, 0, 1, 2, ...
	{
		matB[i-2] = (distanceRel[1]-distanceRel[0]) - (distanceRel[i]-distanceRel[0]); // part of matB
		for (j = 0; j < DIMENSION; j++) // j: index of dimensions, 0(x), 1(y), 2(z)
		{
			matA[i-2][j] = 2*(posAnchors[i][j] - posAnchors[0][j])/(distanceRel[i]-distanceRel[0]) - 2*(posAnchors[1][j] - posAnchors[0][j])/(distanceRel[1]-distanceRel[0]);
			matB[i-2] += ((posAnchors[i][j]-posAnchors[0][j])*(posAnchors[i][j]-posAnchors[0][j]))/(distanceRel[i]-distanceRel[0])
				       - ((posAnchors[1][j]-posAnchors[0][j])*(posAnchors[1][j]-posAnchors[0][j]))/(distanceRel[1]-distanceRel[0]);
		}
	}

	// This part could be A^-1*B (or LS)
	Matrix.Invert(*matA, 2);	// invA
	Matrix.Multiply(*matA, matB, N_ANCHORS-2, DIMENSION, 1, pos);
	*/
}



/* FOR DEBUGGING*/
void DW1000SyncRangingClass::visualizeDatas(byte datas[]) {
	char string[60];
	sprintf(string, "%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X",
					datas[0], datas[1], datas[2], datas[3], datas[4], datas[5], datas[6], datas[7], datas[8], datas[9], datas[10], datas[11], datas[12], datas[13], datas[14], datas[15]);
	Serial.println(string);
}



/* ###########################################################################
 * #### Utils  ###############################################################
 * ######################################################################### */

float DW1000SyncRangingClass::filterValue(float value, float previousValue, uint16_t numberOfElements) {
	
	float k = 2.0f / ((float)numberOfElements + 1.0f);
	return (value * k) + previousValue * (1.0f - k);
}

#endif // defined(TDOA)

