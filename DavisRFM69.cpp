// Driver implementation for HopeRF RFM69W/RFM69HW, Semtech SX1231/1231H used for
// compatibility with the frequency hopped, spread spectrum signals from a Davis Instrument
// wireless Integrated Sensor Suite (ISS)
//
// This is part of the DavisRFM69 library from https://github.com/dekay/DavisRFM69
// (C) DeKay 2014 dekaymail@gmail.com
//
// As I consider this to be a derived work for now from the RFM69W library from LowPowerLab,
// it is released under the same Creative Commons Attrib Share-Alike License
// You are free to use/extend this library but please abide with the CC-BY-SA license:
// http://creativecommons.org/licenses/by-sa/3.0/
//
// In accordance with the CC-BY-SA, many modifications by GitHub user "kobuki".

#include "DavisRFM69.h"
#include "RFM69registers.h"
//#include "TimerOne.h"
//#include "Adafruit_ZeroTimer.h"
#include "PacketFifo.h"

#include <SPI.h>

//#define DAVISRFM69_DEBUG

volatile byte DavisRFM69::DATA[DAVIS_PACKET_LEN];
volatile byte DavisRFM69::_mode = RF69_MODE_INIT;       // current transceiver state
volatile bool DavisRFM69::_packetReceived = false;
volatile byte DavisRFM69::CHANNEL = 0;
volatile byte DavisRFM69::band = 0;
volatile int DavisRFM69::RSSI = 0;   // RSSI measured immediately after payload reception
volatile int16_t DavisRFM69::FEI = 0;
volatile bool DavisRFM69::txMode = false;

volatile uint32_t DavisRFM69::packets = 0;
volatile uint32_t DavisRFM69::lostPackets = 0;
volatile uint32_t DavisRFM69::numResyncs = 0;
volatile uint32_t DavisRFM69::lostStations = 0;
volatile byte DavisRFM69::stationsFound = 0;
volatile byte DavisRFM69::curStation = 0;
volatile byte DavisRFM69::numStations = 0;
volatile byte DavisRFM69::discChannel = 0;
volatile uint32_t DavisRFM69::lastDiscStep;
volatile uint32_t DavisRFM69::int_ticks = 0;
volatile uint32_t rfm69_mode_counts[COUNT_RF69_MODES] = {};
volatile uint32_t rfm69_mode_timer = 0;

enum sm_mode DavisRFM69::mode = SM_IDLE;
PacketFifo DavisRFM69::fifo;
Station *DavisRFM69::stations;
DavisRFM69* DavisRFM69::selfPointer;

//Adafruit_ZeroTimer zt = Adafruit_ZeroTimer(6);

void DavisRFM69::initialize(byte freqBand)
{
  const byte CONFIG[][2] =
  {
    /* 0x01 */ { REG_OPMODE, RF_OPMODE_SEQUENCER_ON | RF_OPMODE_LISTEN_OFF | RF_OPMODE_STANDBY },
    /* 0x02 */ { REG_DATAMODUL, RF_DATAMODUL_DATAMODE_PACKET | RF_DATAMODUL_MODULATIONTYPE_FSK | RF_DATAMODUL_MODULATIONSHAPING_10 }, // Davis uses Gaussian shaping with BT=0.5
    /* 0x03 */ { REG_BITRATEMSB, RF_BITRATEMSB_19200}, // Davis uses a datarate of 19.2 KBPS
    /* 0x04 */ { REG_BITRATELSB, RF_BITRATELSB_19200},
    /* 0x05 */ { REG_FDEVMSB, RF_FDEVMSB_9900}, // Davis uses a deviation of 9.9 kHz
    /* 0x06 */ { REG_FDEVLSB, RF_FDEVLSB_9900},
    /* 0x07 to 0x09 are REG_FRFMSB to LSB. No sense setting them here. Done in main routine.
    /* 0x0B */ { REG_AFCCTRL, RF_AFCLOWBETA_OFF }, // TODO: Should use LOWBETA_ON, but having trouble getting it working
    /* 0x12 */ { REG_PARAMP, RF_PARAMP_25 }, // xxx
    // looks like PA1 and PA2 are not implemented on RFM69W, hence the max output power is 13dBm
    // +17dBm and +20dBm are possible on RFM69HW
    // +13dBm formula: Pout=-18+OutputPower (with PA0 or PA1**)
    // +17dBm formula: Pout=-14+OutputPower (with PA1 and PA2)**
    // +20dBpaym formula: Pout=-11+OutputPower (with PA1 and PA2)** and high power PA settings (section 3.3.7 in datasheet)
    ///* 0x11 */ { REG_PALEVEL, RF_PALEVEL_PA0_ON | RF_PALEVEL_PA1_OFF | RF_PALEVEL_PA2_OFF | RF_PALEVEL_OUTPUTPOWER_11111},
    ///* 0x13 */ { REG_OCP, RF_OCP_ON | RF_OCP_TRIM_95 }, //over current protection (default is 95mA)
    /* 0x18 */ { REG_LNA, RF_LNA_ZIN_50 | RF_LNA_GAINSELECT_AUTO }, // Not sure which is correct!
    // REG_RXBW 50 kHz fixes console retransmit reception but seems worse for SIM transmitters (to be confirmed with more testing)
	// Defaulting to narrow BW, since console retransmits are rarely used - use setBandwidth() to change this
    /* 0x19 */ { REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_20 | RF_RXBW_EXP_4 }, // Use 25 kHz BW (BitRate < 2 * RxBw)
    /* 0x1A */ { REG_AFCBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_20 | RF_RXBW_EXP_3 }, // Use double the bandwidth during AFC as reception
    /* 0x1B - 0x1D These registers are for OOK.  Not used */
    /* 0x1E */ { REG_AFCFEI, RF_AFCFEI_AFCAUTOCLEAR_ON | RF_AFCFEI_AFCAUTO_ON },
    /* 0x1F & 0x20 AFC MSB and LSB values, respectively */
    /* 0x21 & 0x22 FEI MSB and LSB values, respectively */
    /* 0x23 & 0x24 RSSI MSB and LSB values, respectively */
    /* 0x25 */ { REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_01 }, //DIO0 is the only IRQ we're using
    /* 0x26 RegDioMapping2 */
    /* 0x27 RegIRQFlags1 */
    /* 0x28 */ { REG_IRQFLAGS2, RF_IRQFLAGS2_FIFOOVERRUN }, // Reset the FIFOs. Fixes a problem I had with bad first packet.
    /* 0x29 */ { REG_RSSITHRESH, 190 }, // real dBm = -(REG_RSSITHRESH / 2) -> 190 raw = -95 dBm
    /* 0x2a & 0x2b RegRxTimeout1 and 2, respectively */
    /* 0x2c RegPreambleMsb - use zero default */
    /* 0x2d */ { REG_PREAMBLELSB, 0x4 }, // Davis has four preamble bytes 0xAAAAAAAA -- use 6 for TX for this setup
    /* 0x2e */ { REG_SYNCCONFIG, RF_SYNC_ON | RF_SYNC_FIFOFILL_AUTO | RF_SYNC_SIZE_2 | RF_SYNC_TOL_2 },  // Allow a couple erros in the sync word
    /* 0x2f */ { REG_SYNCVALUE1, 0xcb }, // Davis ISS first sync byte. http://madscientistlabs.blogspot.ca/2012/03/first-you-get-sugar.html
    /* 0x30 */ { REG_SYNCVALUE2, 0x89 }, // Davis ISS second sync byte.
    /* 0x31 - 0x36  REG_SYNCVALUE3 - 8 not used */
    /* 0x37 */ { REG_PACKETCONFIG1, RF_PACKET1_FORMAT_FIXED | RF_PACKET1_DCFREE_OFF | RF_PACKET1_CRC_OFF | RF_PACKET1_CRCAUTOCLEAR_OFF | RF_PACKET1_ADRSFILTERING_OFF }, // Fixed packet length and we'll check our own CRC
    /* 0x38 */ { REG_PAYLOADLENGTH, DAVIS_PACKET_LEN }, // Davis sends 8 bytes of payload, including CRC that we check manually.
    //* 0x39 */ { REG_NODEADRS, nodeID }, // Turned off because we're not using address filtering
    //* 0x3a */ { REG_BROADCASTADRS, RF_BROADCASTADDRESS_VALUE }, // Not using this
    /* 0x3b REG_AUTOMODES - Automatic modes are not used in this implementation. */
    /* 0x3c */ { REG_FIFOTHRESH, RF_FIFOTHRESH_TXSTART_FIFOTHRESH | 0x07 }, // TX on FIFO having more than seven bytes
    /* 0x3d */ { REG_PACKETCONFIG2, RF_PACKET2_RXRESTARTDELAY_2BITS | RF_PACKET2_AUTORXRESTART_ON | RF_PACKET2_AES_OFF }, //RXRESTARTDELAY must match transmitter PA ramp-down time (bitrate dependent)
    /* 0x3e - 0x4d  AES Key not used in this implementation */
    /* 0x6F */ { REG_TESTDAGC, RF_DAGC_IMPROVED_LOWBETA0 }, // // TODO: Should use LOWBETA_ON, but having trouble getting it working
    /* 0x71 */ { REG_TESTAFC, 0 }, // AFC Offset for low mod index systems
    {255, 0}
  };

  digitalWrite(_slaveSelectPin, HIGH);
  pinMode(_slaveSelectPin, OUTPUT);
  //SPI.setDataMode(SPI_MODE0);
  //SPI.setBitOrder(MSBFIRST);
  //SPI.setClockDivider(SPI_CLOCK_DIV8); //max speed, except on Due which can run at system clock speed
  SPI.begin();


  do writeReg(REG_SYNCVALUE1, 0xaa); while (readReg(REG_SYNCVALUE1) != 0xaa);
  do writeReg(REG_SYNCVALUE1, 0x55); while (readReg(REG_SYNCVALUE1) != 0x55);
  
  for (byte i = 0; CONFIG[i][0] != 255; i++)
    writeReg(CONFIG[i][0], CONFIG[i][1]);

  setHighPower(_isRFM69HW); //called regardless if it's a RFM69W or RFM69HW
  setMode(RF69_MODE_STANDBY);
  while ((readReg(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00); // Wait for ModeReady
  attachInterrupt(_interruptNum, DavisRFM69::isr0, RISING);

  selfPointer = this;
  userInterrupt = NULL;

  mode = SM_IDLE;

  setBand(freqBand);
  setChannel(discChannel);

  fifo.flush();
  initStations();
  lastDiscStep = micros();

  //Timer1.initialize(2000L); // periodical interrupts every 2 ms for missed packet detection and other checks
  //Timer1.attachInterrupt(DavisRFM69::handleTimerInt, 0);
  
/*
  Serial.println("About to enable timer.");
  Serial.flush();
  delay(500);

  enum status_code sc = zt.configure (TC_CLOCK_PRESCALER_DIV8, 
  				 TC_COUNTER_SIZE_16BIT,
  				 TC_WAVE_GENERATION_NORMAL_FREQ
  				 );
  Serial.print("config status: ");
  Serial.print(sc == STATUS_OK);
  Serial.print(" ");
  Serial.println(sc);

  //zt4.setPeriodMatch(12000, 1, 0);
  zt.setCallback(true, TC_CALLBACK_OVERFLOW, DavisRFM69::handleTimerInt);
  zt.enable(true);
  */
}

void DavisRFM69::stopReceiver() {
  //Timer1.detachInterrupt();
  //zt.enable(false);
  setMode(RF69_MODE_SLEEP);
  mode = SM_IDLE;
}

void DavisRFM69::setStations(Station *_stations, byte n) {
  stations = _stations;
  numStations = n;
}

/**
 * compute a 32bit time difference assuming the first argument
 * happend after the second. (ie accounting for wrap around).
 */
uint32_t DavisRFM69::difftime(uint32_t after, uint32_t before){
	if (after >= before){
		return after - before;
	} else {
		// counter wrapped
		return (0xffffffff - before) + after + 1;
	}
}

/**
 * Called by the main arduino loop, used when not using a timer (handleTimerInt)
 * this will check for lost packets and tune the radio to the proper channel
 * hopefully at the proper time. Added by AMM.
 */
void DavisRFM69::loop(){
  uint8_t i;

  // first see if we have tuned into receive a station previously and failed to actually receive a packet
  if (mode == SM_RECEIVING){

  	// packet was lost
  	if (difftime(micros(), stations[curStation].recvBegan) > (1+stations[curStation].lostPackets)*(LATE_PACKET_THRESH + TUNEIN_USEC)){
//#ifdef DAVISRFM69_DEBUG
		Serial.print(micros());
		Serial.print(": missed packet from station ");
		Serial.print(stations[curStation].id);
		Serial.print(" channel ");
		Serial.println(stations[curStation].channel);
//#endif
		lostPackets++;
		stations[curStation].missedPackets ++;
		stations[curStation].lostPackets ++;
		stations[curStation].lastRx += stations[curStation].interval;
		stations[curStation].channel = nextChannel(stations[curStation].channel);
		
		// lost a station
		if (stations[curStation].lostPackets > RESYNC_THRESHOLD){
//#ifdef DAVISRFM69_DEBUG
			Serial.print(micros());
			Serial.print(": station ");
			Serial.print(stations[curStation].id);
			Serial.println(" is lost.");
//#endif			
			stations[curStation].lostPackets = 0;
			stations[curStation].interval = 0;					

        	lostStations++;
        	stationsFound--;
		}

		curStation = -1;
		mode = SM_IDLE;
		setMode(RF69_MODE_STANDBY);
	} else {
		// waiting to receive and it hasn't timed out yet.
		// do nothing.
		return;
	}
  }

  // next check for any station that's about to transmit
  // if found, tune in
  for (i = 0; i < numStations; i++) {
  	// interval is filled in once we discover a station
  	if (stations[i].interval > 0){
	  	if (difftime(stations[i].lastRx + stations[i].interval, micros()) < ( (1+stations[i].lostPackets)*TUNEIN_USEC)){
#ifdef DAVISRFM69_DEBUG	  		
	  		Serial.print(micros());
	  		Serial.print(": tune to station ");
			Serial.print(stations[i].id);
			Serial.print(" channel ");
			Serial.println(stations[i].channel);
#endif
			stations[i].recvBegan = micros();
	  		setChannel(stations[i].channel);

	  		// we are now set to receive from this station.
	  		mode = SM_RECEIVING;
	  		curStation = i;
	  		return;	  	
	  	} 
  	}
  }

  // if no transmitter is about to transmit, check for any 
  // station that we are not synchonized with, if there are 
  // any stations not synchronized, turn on the receiver and
  // hope we get lucky.
  bool all_sync = true;
  for (i = 0; i < numStations; i++) {  	
  	// unknown stations will have interval of zero, the radio interrupt will 
  	// fill this in if we receive a packet from the given station.
  	if (stations[i].interval == 0){
  		mode = SM_SEARCHING;
  		all_sync = false;
  		if (stations[i].syncBegan == 0){
  			// we have never tried to sync to this station
#ifdef DAVISRFM69_DEBUG  			
  			Serial.print(micros());
  			Serial.print(": begin sync to station ");
  			Serial.print(stations[i].id);
  			Serial.print(" channel ");
  			Serial.println(stations[i].channel);
#endif
  			stations[i].syncBegan = micros();
  			stations[i].progress = 0;
  			setChannel(stations[i].channel);
  		}else if (difftime(micros(), stations[i].syncBegan) > DISCOVERY_STEP){
  			// we tried and failed to sync, try the next channel  			

			stations[i].channel = nextChannel(stations[i].channel);		
#ifdef DAVISRFM69_DEBUG			
			Serial.print(micros());
  			Serial.print(": sync fail, begin sync to station ");
  			Serial.print(stations[i].id);
  			Serial.print(" channel ");
  			Serial.println(stations[i].channel);
#endif
  			stations[i].syncBegan = micros();
  			stations[i].progress = 0;
  			setChannel(stations[i].channel);
  		} else{
  			// we're waiting to hear from this station, don't tune away!
#ifdef DAVISRFM69_DEBUG
  			byte p = difftime(micros(), stations[i].syncBegan) / (DISCOVERY_STEP/100);

  			if (stations[i].progress != p){
  				Serial.print(micros());
	  			Serial.print(": listen progress ");
	  			Serial.print(p);
	  			Serial.println("\%");	 
	  			stations[i].progress = p;			
	  		}
#endif	  		
  			break;
  		}
  	}
  }

  if (all_sync) {
  	mode = SM_SYNCHRONIZED;

  	// if we got here, no stations are about to TX and all stations are in sync,
  	// we can disable our radio to save power.
  	if (_mode != RF69_MODE_SLEEP){
#ifdef DAVISRFM69_DEBUG  	  
  	  Serial.print(micros());
  	  Serial.println(": Nothing to do, going to sleep");
#endif  	  
  	  setMode(RF69_MODE_SLEEP);
  	}
  }
}

/*
// Handle missed packets. Called from Timer1 ISR every 2 ms
void DavisRFM69::handleTimerInt() {

  uint32_t lastRx = micros();
  bool readjust = false;
  
  int_ticks ++;
  if (stations[curStation].interval > 0
	  && stations[curStation].lastRx + stations[curStation].interval - lastRx < DISCOVERY_GUARD
      && CHANNEL != stations[curStation].channel) {
  	Serial.println("op1");
    selfPointer->setChannel(stations[curStation].channel);
	return;
  }

  if (lastRx - lastDiscStep > DISCOVERY_STEP) {
    discChannel = selfPointer->nextChannel(discChannel);
    lastDiscStep = lastRx;
  }

  // find and adjust 'last seen' rx time on all stations with older reception timestamp than their period + threshold
  // that is, find missed packets
  for (byte i = 0; i < numStations; i++) {
    if (stations[i].interval > 0 && (lastRx - stations[i].lastRx) > stations[i].interval + LATE_PACKET_THRESH) {
      if (stations[curStation].active) lostPackets++;
      stations[i].lostPackets++;
      stations[i].missedPackets++;
      if (stations[i].lostPackets > RESYNC_THRESHOLD) {
        stations[i].numResyncs++;
        stations[i].interval = 0;
        stations[i].lostPackets = 0;
        lostStations++;
        stationsFound--;
        if (lostStations == numStations) {
        	Serial.println("lost stations.");
          numResyncs++;
          stationsFound = 0;
          lostStations = 0;
          selfPointer->initStations();
          selfPointer->setChannel(discChannel);
          return;
        }
      } else {
        stations[i].lastRx += stations[i].interval; // when packet should have been received
        stations[i].channel = selfPointer->nextChannel(stations[i].channel); // skip station's next channel in hop seq
        readjust = true;
      }
    }
  }

  if (readjust) {
  	Serial.println("readjust.");
    selfPointer->nextStation();
    selfPointer->setChannel(stations[curStation].channel);
  }
}
*/

// Handle received packets, called from RFM69 ISR
void DavisRFM69::handleRadioInt() {

  uint32_t lastRx = micros();
  uint16_t rxCrc = word(DATA[6], DATA[7]);  // received CRC
  uint16_t calcCrc = DavisRFM69::crc16_ccitt(DATA, 6);  // calculated CRC

  bool repeaterCrcTried = false;

  // repeater packets checksum bytes (0..5) and (8..9), so try this at mismatch
  if (calcCrc != rxCrc) {
    calcCrc = DavisRFM69::crc16_ccitt(DATA + 8, 2, calcCrc);
	repeaterCrcTried = true;
  }

  delayMicroseconds(POST_RX_WAIT); // we need this, no idea why, but makes reception almost perfect
                                   // probably needed by the module to settle something after RX

  // fifo.queue((byte*)DATA, CHANNEL, -RSSI, FEI, lastRx - stations[curStation].lastRx);

  // packet passed crc?
  if (calcCrc == rxCrc && rxCrc != 0) {

    // station id is byte 0:0-2
    byte id = DATA[0] & 7;
    int stIx = findStation(id);

    // if we have no station cofigured for this id (at all; can still be be !active), ignore the packet
	// OR packet passed the repeater crc check, but no repeater is set for the station
	// OR packet passed the normal crc check, and repeater is set for the station
    if (stIx < 0
        || (repeaterCrcTried && stations[stIx].repeaterId == 0)
        || (!repeaterCrcTried && stations[stIx].repeaterId != 0)) {
      setChannel(CHANNEL);
      return;
    }

    if (stationsFound < numStations && stations[stIx].interval == 0) {
      stations[stIx].interval = (41 + id) * 1000000 / 16; // Davis' official tx interval in us

      stationsFound++;
#ifdef DAVISRFM69_DEBUG      
      Serial.print("found station ");
      Serial.print(stIx);
      Serial.print(" interval ");
      Serial.println(stations[stIx].interval);
#endif
      if (lostStations > 0) lostStations--;
    }
    
    packets++;	
    if (stations[stIx].active) {

	  stations[stIx].packets++;
      fifo.queue((byte*)DATA, CHANNEL, -RSSI, FEI, stations[curStation].lastSeen > 0 ? lastRx - stations[curStation].lastSeen : 0);
    }
	
	stations[stIx].earlyAmt = difftime(lastRx, stations[stIx].recvBegan);
    stations[stIx].lostPackets = 0;
    stations[stIx].lastRx = stations[stIx].lastSeen = lastRx;
    stations[stIx].channel = nextChannel(CHANNEL);
#ifdef DAVISRFM69_DEBUG
    Serial.print("early amt = ");
    Serial.println(stations[stIx].earlyAmt);
#endif
    // no longer waiting to RX (if we were at all anwyay)
    mode = SM_IDLE;
    // standby the radio
    setMode (RF69_MODE_STANDBY);

  } else {
  	// bad CRC, go back to RX on this channel
    setChannel(CHANNEL); // this always has to be done somewhere right after reception, even for ignored/bogus packets
  }

}

// Calculate the next hop of the specified channel
byte DavisRFM69::nextChannel(byte channel) {
  return ++channel % getBandTabLength();
}

// Find the station index in stations[] for station expected to tx the earliest and update curStation
void DavisRFM69::nextStation() {
  uint32_t earliest = 0xffffffff;
  uint32_t now = micros();
  for (int i = 0; i < numStations; i++) {
    uint32_t current = stations[i].lastRx + stations[i].interval - now;
    if (stations[i].interval > 0 && current < earliest) {
      earliest = current;
      curStation = i;
    }
  }
}

// Find station index in stations[] for a station ID (-1 if doesn't exist)
int DavisRFM69::findStation(byte id) {
  for (byte i = 0; i < numStations; i++) {
    if (stations[i].id == id) return i;
  }
  return -1;
}

// Reset station array to safe defaults
void DavisRFM69::initStations() {
  for (byte i = 0; i < numStations; i++) {
    stations[i].channel = 0;
    stations[i].lastRx = 0;
    stations[i].interval = 0;
    stations[i].lostPackets = 0;
    stations[i].lastRx = 0;
    stations[i].lastSeen = 0;
    stations[i].packets = 0;
    stations[i].missedPackets = 0;
    stations[i].syncBegan = 0;
  }
}

void DavisRFM69::interruptHandler() {
  RSSI = readRSSI();  // Read up front when it is most likely the carrier is still up
  if (_mode == RF69_MODE_RX && (readReg(REG_IRQFLAGS2) & RF_IRQFLAGS2_PAYLOADREADY))
  {
    FEI = word(readReg(REG_FEIMSB), readReg(REG_FEILSB));
    setMode(RF69_MODE_STANDBY);
    select();   // Select RFM69 module, disabling interrupts
    SPI.transfer(REG_FIFO & 0x7f);

    for (byte i = 0; i < DAVIS_PACKET_LEN; i++) DATA[i] = reverseBits(SPI.transfer(0));

    _packetReceived = true;

    handleRadioInt();

    unselect();  // Unselect RFM69 module, enabling interrupts
  }
}

bool DavisRFM69::canSend()
{
  if (_mode == RF69_MODE_RX && readRSSI() < CSMA_LIMIT) //if signal stronger than -100dBm is detected assume channel activity
  {
    setMode(RF69_MODE_STANDBY);
    return true;
  }
  return false;
}

// IMPORTANT: make sure buffer is at least 6 bytes
void DavisRFM69::send(const void* buffer)
{
  writeReg(REG_PACKETCONFIG2, (readReg(REG_PACKETCONFIG2) & 0xFB) | RF_PACKET2_RXRESTART); // avoid RX deadlocks
  sendFrame(buffer);
}

// IMPORTANT: make sure buffer is at least 6 bytes
void DavisRFM69::sendFrame(const void* buffer)
{
  setMode(RF69_MODE_STANDBY); //turn off receiver to prevent reception while filling fifo
  while ((readReg(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00); // Wait for ModeReady
  writeReg(REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_00); // DIO0 is "Packet Sent"

  // calculate crc on first 6 bytes (no repeater info)
  uint16_t crc = crc16_ccitt((volatile byte *)buffer, 6);
  // write to FIFO
  select();
  SPI.transfer(REG_FIFO | 0x80);

  // The following byte sequence is an attempt to emulate the original Davis packet structure.

  // carrier on/start
  SPI.transfer(0xff);
  SPI.transfer(0xff);
  SPI.transfer(0xff);
  SPI.transfer(0xff);

  // preamble
  SPI.transfer(0xaa);
  SPI.transfer(0xaa);
  SPI.transfer(0xaa);
  SPI.transfer(0xaa);

  // sync word
  SPI.transfer(0xcb);
  SPI.transfer(0x89);

  // transmit first 6 byte of the buffer
  for (byte i = 0; i < 6; i++)
    SPI.transfer(reverseBits(((byte*)buffer)[i]));

  // transmit crc of first 6 bytes
  SPI.transfer(reverseBits(crc >> 8));
  SPI.transfer(reverseBits(crc & 0xff));

  // transmit dummy repeater info (always 0xff, 0xff for a normal packet without repeaters)
  SPI.transfer(0xff);
  SPI.transfer(0xff);

  unselect();

  /* no need to wait for transmit mode to be ready since its handled by the radio */
  setMode(RF69_MODE_TX);
  while (digitalRead(_interruptPin) == 0); //wait for DIO0 to turn HIGH signalling transmission finish
  setMode(RF69_MODE_STANDBY);
}

void DavisRFM69::setChannel(byte channel)
{
	//Serial.print("CH->>");
	//Serial.println(channel);
  CHANNEL = channel;
  if (CHANNEL > bandTabLengths[band] - 1) CHANNEL = 0;
  writeReg(REG_FRFMSB, pgm_read_byte(&bandTab[band][CHANNEL][0]));
  writeReg(REG_FRFMID, pgm_read_byte(&bandTab[band][CHANNEL][1]));
  writeReg(REG_FRFLSB, pgm_read_byte(&bandTab[band][CHANNEL][2]));
  if (!txMode) receiveBegin();
}

void DavisRFM69::hop()
{
  setChannel(++CHANNEL);
}

// The data bytes come over the air from the ISS least significant bit first. Fix them as we go. From
// http://www.ocf.berkeley.edu/~wwu/cgi-bin/yabb/YaBB.cgi?board=riddles_cs;action=display;num=1103355188
byte DavisRFM69::reverseBits(byte b)
{
  b = ((b & 0b11110000) >>4 ) | ((b & 0b00001111) << 4);
  b = ((b & 0b11001100) >>2 ) | ((b & 0b00110011) << 2);
  b = ((b & 0b10101010) >>1 ) | ((b & 0b01010101) << 1);

  return(b);
}

// Davis CRC calculation from http://www.menie.org/georges/embedded/
uint16_t DavisRFM69::crc16_ccitt(volatile byte *buf, byte len, uint16_t initCrc)
{
  uint16_t crc = initCrc;
  while( len-- ) {
    int i;
    crc ^= *(char *)buf++ << 8;
    for( i = 0; i < 8; ++i ) {
      if( crc & 0x8000 )
        crc = (crc << 1) ^ 0x1021;
      else
        crc = crc << 1;
    }
  }
  return crc;
}

void DavisRFM69::setMode(byte newMode)
{
  //Serial.println(newMode);
  if (newMode == _mode) return;

  if (_mode < COUNT_RF69_MODES){
  	rfm69_mode_counts[_mode] += difftime(micros(), rfm69_mode_timer);
  	rfm69_mode_timer = micros();
#ifdef DAVISRFM69_DEBUG  
	  Serial.print(RFM69_MODE_STRINGS[_mode]);
	  Serial.print(" -> ");
	  Serial.println(RFM69_MODE_STRINGS[newMode]);
#endif	  
  }else{
  		Serial.println("INVALID MODE CHANGE.");
  }
  switch (newMode) {
    case RF69_MODE_TX:
      writeReg(REG_OPMODE, (readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_TRANSMITTER);
      if (_isRFM69HW) setHighPowerRegs(true);
      break;
    case RF69_MODE_RX:
      writeReg(REG_OPMODE, (readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_RECEIVER);
      if (_isRFM69HW) setHighPowerRegs(false);
      break;
    case RF69_MODE_SYNTH:
      writeReg(REG_OPMODE, (readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_SYNTHESIZER);
      break;
    case RF69_MODE_STANDBY:
      writeReg(REG_OPMODE, (readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_STANDBY);
      break;
    case RF69_MODE_SLEEP:
      writeReg(REG_OPMODE, (readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_SLEEP);
      break;
    default: return;
  }

  // we are using packet mode, so this check is not really needed
  // but waiting for mode ready is necessary when going from sleep because the FIFO may not be immediately available from previous mode
  while (_mode == RF69_MODE_SLEEP && (readReg(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00); // Wait for ModeReady

  _mode = newMode;
  //Serial.print("Mode set to ");
  //Serial.println(_mode);
}

void DavisRFM69::sleep() {
  setMode(RF69_MODE_SLEEP);
}

void DavisRFM69::isr0() { selfPointer->interruptHandler(); }

void DavisRFM69::receiveBegin() {
  _packetReceived = false;
  if (readReg(REG_IRQFLAGS2) & RF_IRQFLAGS2_PAYLOADREADY)
    writeReg(REG_PACKETCONFIG2, (readReg(REG_PACKETCONFIG2) & 0xFB) | RF_PACKET2_RXRESTART); // avoid RX deadlocks

  writeReg(REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_01); //set DIO0 to "PAYLOADREADY" in receive mode
  setMode(RF69_MODE_RX);
}

bool DavisRFM69::receiveDone() {
  return _packetReceived;
}

void DavisRFM69::setRssiThreshold(int rssiThreshold) {
  writeReg(REG_RSSIVALUE, abs(rssiThreshold) << 1);
}

void DavisRFM69::setRssiThresholdRaw(int rssiThresholdRaw) {
  writeReg(REG_RSSIVALUE, rssiThresholdRaw);
}

int DavisRFM69::readRSSI(bool forceTrigger) {
  int rssi = 0;
  if (forceTrigger)
  {
    //RSSI trigger not needed if DAGC is in continuous mode
    writeReg(REG_RSSICONFIG, RF_RSSI_START);
    while ((readReg(REG_RSSICONFIG) & RF_RSSI_DONE) == 0x00); // Wait for RSSI_Ready
  }
  rssi = -readReg(REG_RSSIVALUE);
  rssi >>= 1;
  return rssi;
}

byte DavisRFM69::readReg(byte addr)
{
  select();
  SPI.transfer(addr & 0x7F);
  byte regval = SPI.transfer(0);
  unselect();
  return regval;
}

void DavisRFM69::writeReg(byte addr, byte value)
{
  select();
  SPI.transfer(addr | 0x80);
  SPI.transfer(value);
  unselect();
}

/// Select the transceiver
void DavisRFM69::select() {
  noInterrupts();
  digitalWrite(_slaveSelectPin, LOW);
}

/// Unselect the transceiver chip
void DavisRFM69::unselect() {
  digitalWrite(_slaveSelectPin, HIGH);
  interrupts();
}

void DavisRFM69::setHighPower(bool onOff) {
  _isRFM69HW = onOff;
  writeReg(REG_OCP, _isRFM69HW ? RF_OCP_OFF : RF_OCP_ON);
  if (_isRFM69HW) //turning ON
    writeReg(REG_PALEVEL, (readReg(REG_PALEVEL) & 0x1F) | RF_PALEVEL_PA1_ON | RF_PALEVEL_PA2_ON); //enable P1 & P2 amplifier stages
    else
      writeReg(REG_PALEVEL, RF_PALEVEL_PA0_ON | RF_PALEVEL_PA1_OFF | RF_PALEVEL_PA2_OFF | _powerLevel); //enable P0 only
}

void DavisRFM69::setHighPowerRegs(bool onOff) {
  writeReg(REG_TESTPA1, onOff ? 0x5D : 0x55);
  writeReg(REG_TESTPA2, onOff ? 0x7C : 0x70);
}

void DavisRFM69::setCS(byte newSPISlaveSelect) {
  _slaveSelectPin = newSPISlaveSelect;
  pinMode(_slaveSelectPin, OUTPUT);
}

//for debugging
void DavisRFM69::readAllRegs()
{
  byte regVal;

  for (byte regAddr = 1; regAddr <= 0x4F; regAddr++)
  {
    select();
    SPI.transfer(regAddr & 0x7f); // send address + r/w bit
    regVal = SPI.transfer(0);
    unselect();

    Serial.print(regAddr, HEX);
    Serial.print(" - ");
    Serial.print(regVal,HEX);
    Serial.print(" - ");
    Serial.println(regVal,BIN);
  }
  unselect();
}

byte DavisRFM69::readTemperature(byte calFactor)  //returns centigrade
{
  setMode(RF69_MODE_STANDBY);
  writeReg(REG_TEMP1, RF_TEMP1_MEAS_START);
  while ((readReg(REG_TEMP1) & RF_TEMP1_MEAS_RUNNING)) Serial.print('*');
  return ~readReg(REG_TEMP2) + COURSE_TEMP_COEF + calFactor; //'complement'corrects the slope, rising temp = rising val
}                             // COURSE_TEMP_COEF puts reading in the ballpark, user can add additional correction

void DavisRFM69::rcCalibration()
{
  writeReg(REG_OSC1, RF_OSC1_RCCAL_START);
  while ((readReg(REG_OSC1) & RF_OSC1_RCCAL_DONE) == 0x00);
}

void DavisRFM69::setTxMode(bool txMode)
{
  DavisRFM69::txMode = txMode;
  if (txMode) {
    writeReg(REG_PREAMBLELSB, 0);
    writeReg(REG_SYNCCONFIG, RF_SYNC_OFF);
    // +10 = 4 bytes "carrier" (0xff) + 4 bytes preamble (0xaa) + 2 bytes sync
    writeReg(REG_PAYLOADLENGTH, DAVIS_PACKET_LEN + 10);
    writeReg(REG_FIFOTHRESH, RF_FIFOTHRESH_TXSTART_FIFOTHRESH | (DAVIS_PACKET_LEN + 10 - 1));
  } else {
    writeReg(REG_PREAMBLELSB, 4);
    writeReg(REG_SYNCCONFIG, RF_SYNC_ON | RF_SYNC_FIFOFILL_AUTO | RF_SYNC_SIZE_2 | RF_SYNC_TOL_2);
    writeReg(REG_PAYLOADLENGTH, DAVIS_PACKET_LEN);
    writeReg(REG_FIFOTHRESH, RF_FIFOTHRESH_TXSTART_FIFOTHRESH | (DAVIS_PACKET_LEN - 1));
  }
}

void DavisRFM69::setBand(byte newBand)
{
  band = newBand;
}

void DavisRFM69::setBandwidth(byte bw)
{
  switch (bw) {
    case RF69_DAVIS_BW_NARROW:
	  writeReg(REG_RXBW,  RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_20 | RF_RXBW_EXP_4); // Use 25 kHz BW (BitRate < 2 * RxBw)
	  writeReg(REG_AFCBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_20 | RF_RXBW_EXP_3); // Use double the bandwidth during AFC as reception
      break;
    case RF69_DAVIS_BW_WIDE:
      // REG_RXBW 50 kHz fixes console retransmit reception but seems worse for SIM transmitters (to be confirmed with more testing)
	  writeReg(REG_RXBW,  RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_20 | RF_RXBW_EXP_3); // Use 50 kHz BW (BitRate < 2 * RxBw)
	  writeReg(REG_AFCBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_20 | RF_RXBW_EXP_2); // Use double the bandwidth during AFC as reception
      break;
    default:
      return;
  }
}

byte DavisRFM69::getBandTabLength()
{
  return bandTabLengths[band];
}
