#include <Arduino.h>

#include <SPI.h>
//#include <EEPROM.h>

#include <Wire.h>

#include <stdint.h>

#include "DavisRFM69.h"
#include "PacketFifo.h"

#define LED 13
#define SERIAL_BAUD 19200

DavisRFM69 radio(8, 3, true, 3);

// id, type, active
Station stations[1] = {
  { .id = 0,
    .type =  STYPE_VUE,
    .active = true}
};

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(1000);
  
  pinMode(LED, OUTPUT); 
  digitalWrite(LED, LOW);
  
  radio.setStations(stations, 1);
  radio.initialize(FREQ_BAND_US);
  //radio.setBandwidth(RF69_DAVIS_BW_NARROW);
  radio.setBandwidth(RF69_DAVIS_BW_WIDE);
  
  Serial.println("Boot complete!");
}


void loop() {

/*
 static uint32_t last_print = 0;

  if (millis() - last_print > 1000){
    // print performance DC stats
    uint32_t sum = 0;
    for (uint8_t i = 0; i < COUNT_RF69_MODES; i++){
      sum += radio.rfm69_mode_counts[i];
    }
    for (uint8_t i = 0; i < COUNT_RF69_MODES; i++){
      Serial.print(RFM69_MODE_STRINGS[i]);
      Serial.print(": ");
      Serial.print(radio.rfm69_mode_counts[i]);
      Serial.print(" ");
      Serial.print(radio.rfm69_mode_counts[i]/ (sum / 100.0));
      Serial.println("\%");      
    }

    last_print = millis(); 
  }
*/
  if (radio.fifo.hasElements()) {
    decode_packet(radio.fifo.dequeue());
  }

  if (radio.mode == SM_RECEIVING) {
    digitalWrite(LED, HIGH);
  } else if (radio.mode == SM_SEARCHING){
    Blink(LED, 15);
    Blink(LED, 15);
    Blink(LED, 15);
    delay(100);
  }else{
    digitalWrite(LED, LOW);
  }

  radio.loop();
    
}

void Blink(byte PIN, int DELAY_MS)
{
  pinMode(PIN, OUTPUT);
  digitalWrite(PIN,HIGH);
  delay(DELAY_MS);
  digitalWrite(PIN,LOW);
}

void print_value(char* vname, char* value, const __FlashStringHelper* sep) {
  Serial.print(vname); Serial.print(F(":")); Serial.print(value); Serial.print(sep);
}

void print_value(char* vname, int value, const __FlashStringHelper* sep) {
  Serial.print(vname); Serial.print(F(":")); Serial.print(value); Serial.print(sep);
}

void print_value(char* vname, float value, const __FlashStringHelper* sep) {
  Serial.print(vname); Serial.print(F(":")); Serial.print(value, 1); Serial.print(sep);
}

void print_value(char* vname, long value, const __FlashStringHelper* sep) {
  Serial.print(vname); Serial.print(F(":")); Serial.print(value); Serial.print(sep);
}

void print_value(char* vname, uint32_t value, const __FlashStringHelper* sep) {
  Serial.print(vname); Serial.print(F(":")); Serial.print(value); Serial.print(sep);
}

void decode_packet(RadioData* rd) {

  // for more about the protocol see:
  // https://github.com/dekay/DavisRFM69/wiki/Message-Protocol
  int val;
  byte* packet = rd->packet;

  Serial.print(F("raw:"));
  printHex(packet, 10);
  Serial.print(F(", "));
  print_value("station", packet[0] & 0x7, F(", "));
  

  Serial.print(F("packets:"));
  Serial.print(radio.packets);
  Serial.print('/');
  Serial.print(radio.lostPackets);
  Serial.print('/');
  Serial.print((float)(radio.packets * 100.0 / (radio.packets + radio.lostPackets)));
  Serial.print(F(", "));

  print_value("channel", rd->channel, F(", "));
  print_value("rssi", -rd->rssi, F(", "));

  print_value("batt", (char*)(packet[0] & 0x8 ? "err" : "ok"), F(", "));

  // All packet payload values are printed unconditionally, either properly
  // calculated or flagged with a special "missing sensor" value, mostly -1.
  // It's the CPE's responsibility to interpret our output accordingly.

  byte id = radio.DATA[0] & 7;
  int stIx = radio.findStation(id);

  // wind data is present in every packet, windd == 0 (packet[2] == 0) means there's no anemometer
  if (packet[2] != 0) {
    if (stations[stIx].type == STYPE_VUE) {
      val = (packet[2] << 1) | (packet[4] & 2) >> 1;
      val = round(val * 360 / 512);
    } else {
      val = 9 + round((packet[2] - 1) * 342.0 / 255.0);
    }
  } else {
    val = 0;
  }
  print_value("windv", packet[1], F(", "));
  print_value("winddraw", packet[2], F(", "));
  print_value("windd", val, F(", "));

  switch (packet[0] >> 4) {

    case VP2P_UV:
      val = word(packet[3], packet[4]) >> 6;
      if (val < 0x3ff) {
        print_value("uv", (float)(val / 50.0), F(", "));
      } else {
        print_value("uv", -1, F(", "));
      }
      break;

    case VP2P_SOLAR:
      val = word(packet[3], packet[4]) >> 6;
      if (val < 0x3fe) {
        print_value("solar", (float)(val * 1.757936), F(", "));
      } else {
        print_value("solar", -1, F(", "));
      }
      break;

    case VP2P_RAIN:
      if (packet[3] == 0x80) {
        print_value("rain", -1, F(", "));
      } else {
        print_value("rain", packet[3], F(", "));
      }
      break;

    case VP2P_RAINSECS:
      // light rain:  byte4[5:4] as value[9:8] and byte3[7:0] as value[7:0] - 10 bits total
      // strong rain: byte4[5:4] as value[5:4] and byte3[7:4] as value[3:0] - 6 bits total
      val = (packet[4] & 0x30) << 4 | packet[3];
      if (val == 0x3ff) {
        print_value("rainsecs", -1, F(", "));
      } else {
        if ((packet[4] & 0x40) == 0) val >>= 4; // packet[4] bit 6: strong == 0, light == 1
        print_value("rainsecs", val, F(", "));
      }
      break;

    case VP2P_TEMP:
      //if (packet[3] == 0xff) {
//        print_value("temp", -100, F(", "));
      //} else {
      {
        //val = (int)packet[3] << 4 | packet[4] >> 4;
        //val = (packet[3]* 256 + packet[4]) / 160;
        val = ((int16_t)((packet[3]<<8) | packet[4])) / 16;
        print_value("temp", (float)(val / 10.0), F(", "));
      }
      break;

    case VP2P_HUMIDITY:
      val = ((packet[4] >> 4) << 8 | packet[3]) / 10; // 0 -> no sensor
      print_value("rh", (float)val, F(", "));
      break;

    case VP2P_WINDGUST:
      print_value("windgust", packet[3], F(", "));
      // gustref is the index of the last message 9 packet containing the gust (max wind speed).
      if (packet[3] != 0) {
        print_value("gustref", packet[5] & 0xf0 >> 4, F(", "));
      }
      break;

    case VP2P_SOIL_LEAF:
      // currently not processed but algorithm is known
      // see https://github.com/matthewwall/weewx-meteostick/blob/master/bin/user/meteostick.py
      print_value("soilleaf", -1, F(", "));
      break;

    case VUEP_VCAP:           
      val = (packet[3] << 2) | (packet[4] & 0xc0) >> 6;    
      print_value("vcap", (float)(val / 100.0), F(", "));
      break;

    case VUEP_VSOLAR:
      val = (packet[3] << 2) | (packet[4] & 0xc0) >> 6;
      print_value("vsolar", (float)(val / 100.0), F(", "));
  }

  print_value("fei", round(rd->fei * RF69_FSTEP / 1000), F(", "));
  print_value("delta", rd->delta, F(""));

  Serial.println();
}

void printHex(volatile byte* packet, byte len) {
  for (byte i = 0; i < len; i++) {
    if (!(packet[i] & 0xf0)) Serial.print('0');
    Serial.print(packet[i], HEX);
    if (i < len - 1) Serial.print('-');
  }
}
