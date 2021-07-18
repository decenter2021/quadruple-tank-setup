/* arduino-quadruple-tank.ino
 * Main file for Arduino Nano of the Quadruple Tank Process for the
 * interface with Simulink via Serial Communication.
 * It receives PWM signal to input to the pumps and brodcasts the
 * voltage values read from the water level sensors.
 * Authors: Leonardo Pedroso, Pedro Batista
 * Institute for Systems and Robotics, Instituto Superior Técnico, 
 * Lisbon, Portugal*/

 /* Developed for project DECENTER (https://decenter2021.github.io/) under MIT license
  * Copyright (c) 2021 DECENTER
  * Permission is hereby granted, free of charge, to any person obtaining 
  * a copy of this software and associated documentation files (the 
  * “Software”), to deal in the Software without restriction, including 
  * without limitation the rights to use, copy, modify, merge, publish, 
  * distribute, sublicense, and/or sell copies of the Software, and to 
  * permit persons to whom the Software is furnished to do so, subject 
  * to the following conditions:
  * The above copyright notice and this permission notice shall be included 
  * in all copies or substantial portions of the Software.
  * THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS 
  * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
  * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
  * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
  * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING 
  * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS 
  * IN THE SOFTWARE.
  */
 
// Include header file
#include "arduino-quadruple-tank.h"

// ---------- Global variables ----------
// Data broadcast and received from simulink is stored on struct quadrupleData
quadrupleData quadruple;
// Buffer of a single 16 bit word for I2C interface with ADCs
uint16_t bufADC;
// Buffer of a single 8 bit word for serial interface with simulink
uint8_t bufSerial = 0;
// Time of previous received query (used to stop pumps if Simulink times out)
unsigned long prevCom = 0;

// ---------- Setup (runs on startup) ----------
void setup() {
  Serial.begin(9600); // Init serial communication @ 9600 baud
  Wire.begin(); // Init I2C bus for communication with ADCs
  setupPWM10bit(); // Setup 10 bit PWM for pins 9 and 10 (i.e. the actuation pumps)
  pumpInit(); // Setup arduino pins to control the pumps
}

// ---------- Loop (runs on an infinite loop) ----------
/* Information is shared in simulink in the binblock format:
 * https://www.mathworks.com/help/instrument/binblockwrite.html
 * | HEADER | NUMBER OF DIGITS OF #bytes | #bytes | Data in Big Endian | 
 * For water level query:
 * Receives: Header HEADER_REQUEST
 * Brodcasts: | HEADER | '2' | '10' | output_sensor_1 ... output_sensor_4 output_Vcc (2 bytes *5 = 10 bytes)
 * For pump input query:
 * Receives:  | HEADER | '1' | '6' | u1 u2 d3 d4 (2 bytes *2 + 1 byte *2 = 6 bytes)
 */
void loop() {
  // Listen constantly for a query
  if(Serial.available()){
    bufSerial = (uint8_t) Serial.read(); // Read header of request
    if(bufSerial == HEADER_REQUEST){ // If water level is requested
      readTankLevel(); // Read water level sensors
      levelDataSend(); // Broadcast data to simulink
      prevCom = millis(); // Save communication time
    }else if(bufSerial == HEADER){ // If inputs to pumps are received
      bufSerial = 0; // Init buffer to serve as a counter
      while(bufSerial<8){ // Iterate through each of the 6 bytes in the binblock data
        if(Serial.available()){ // Check if nest byte was received
          if(bufSerial <2){ // Ignore the metadata (first 2 bytes) ('16')
            Serial.read();
          }else if(bufSerial == 2){ // Read most significant byte of u[0] (Big Endian format is used for communication)
            quadruple.u[0] = (uint16_t) Serial.read();
            quadruple.u[0] = quadruple.u[0] << 8;
          }else if(bufSerial == 3){ // Read least significant byte of u[0]
            quadruple.u[0] = quadruple.u[0] | (uint16_t) Serial.read();
          }else if(bufSerial == 4){ // Read most significant byte of u[1] (Big Endian format is used for communication)
            quadruple.u[1] = (uint16_t) Serial.read();
            quadruple.u[1] = quadruple.u[1] << 8;
          }else if(bufSerial == 5){ // Read least significant byte of u[1] 
            quadruple.u[1] = quadruple.u[1] | (uint16_t) Serial.read();
          }else if(bufSerial == 6){ // Read d[0]
            quadruple.d[0] = (uint8_t) Serial.read();
          }else if(bufSerial == 7){ // Read d[1]
            quadruple.d[1] = (uint8_t) Serial.read();
          }
          bufSerial ++; // Increment counter
        }
      }
      pumpWrite(); // Write input that was just received to the pumps   
    }  
  }
  // Check if connection was lost
  if(millis()-prevCom > TIMEOUT ){
    quadruple.u[0] = 0; // Stop pumps
    quadruple.u[1] = 0;
    quadruple.d[0] = 0;
    quadruple.d[1] = 0;
    pumpWrite();
  }
}

// ---------- Read water level ----------
/* This function reads the four output voltages of the water level sensors
 * and the Vcc and writes them to the struct quadruple */
void readTankLevel(){
  // Read A0
  I2Cdev::writeWord(ADC0_ADDR,ADC_REG_CONFIG,A0_CONFIG);
  while(1){ // Wait until sample is available
    I2Cdev::readBitW(ADC0_ADDR,ADC_REG_CONFIG,15,&bufADC);
    if(bufADC){ // When bit 15 is true (sample avialable) get sample from ADC
      I2Cdev::readWord(ADC0_ADDR,ADC_REG_CONV,&quadruple.v[0]);
      break;
    } 
  }
  // Read A1
  I2Cdev::writeWord(ADC0_ADDR,ADC_REG_CONFIG,A1_CONFIG);
  while(1){ // Wait until sample is available
    I2Cdev::readBitW(ADC0_ADDR,ADC_REG_CONFIG,15,&bufADC);
    if(bufADC){ // When bit 15 is true (sample avialable) get sample from ADC
      I2Cdev::readWord(ADC0_ADDR,ADC_REG_CONV,&quadruple.v[1]);
      break;
    } 
  }
  // Read A2
  I2Cdev::writeWord(ADC0_ADDR,ADC_REG_CONFIG,A2_CONFIG);
  while(1){ // Wait until sample is available
    I2Cdev::readBitW(ADC0_ADDR,ADC_REG_CONFIG,15,&bufADC);
    if(bufADC){ // When bit 15 is true (sample avialable) get sample from ADC
      I2Cdev::readWord(ADC0_ADDR,ADC_REG_CONV,&quadruple.v[2]);
      break;
    } 
  }
  // Read A3
  I2Cdev::writeWord(ADC0_ADDR,ADC_REG_CONFIG,A3_CONFIG);
  while(1){ // Wait until sample is available
    I2Cdev::readBitW(ADC0_ADDR,ADC_REG_CONFIG,15,&bufADC);
    if(bufADC){ // When bit 15 is true (sample avialable) get sample from ADC
      I2Cdev::readWord(ADC0_ADDR,ADC_REG_CONV,&quadruple.v[3]);
      break;
    } 
  }
  // Read A0 - Vcc
  I2Cdev::writeWord(ADC1_ADDR,ADC_REG_CONFIG,A0_CONFIG);
  while(1){ // Wait until sample is available
    I2Cdev::readBitW(ADC1_ADDR,ADC_REG_CONFIG,15,&bufADC);
    if(bufADC){ // When bit 15 is true (sample avialable) get sample from ADC
      I2Cdev::readWord(ADC1_ADDR,ADC_REG_CONV,&quadruple.v[4]);
      break;
    } 
  }
 }

// ---------- Control pumps ----------
// This function initilizes the io pins that control the pumps
void pumpInit(){
  pinMode(PUMP12_ENB, OUTPUT); // Init enable pin of actuation pumps (active high)
  pinMode(PUMP34_ENB, OUTPUT); // Init enable pin of disturbance pumps (active high)
  digitalWrite(PUMP12_ENB,HIGH);  // Enable actuation pumps
  digitalWrite(PUMP34_ENB,HIGH); // Enable disturnbance pumps
}

// This function writes the received input to the pumps
void pumpWrite(){
  analogWrite10bit(PUMP1_CTRL,quadruple.u[0]); // Write 10 bit pwm to pump 1
  analogWrite10bit(PUMP2_CTRL,quadruple.u[1]); // Write 10 bit pwm to pump 2
  analogWrite(PUMP3_CTRL,quadruple.d[0]); // Write 8 bit pwm to disturbance pump 1
  analogWrite(PUMP4_CTRL,quadruple.d[1]); // Write 8 bit PWM to disturbance pump 2
}

// ---------- Broadcast water level data ----------
/* This function broadcasts the water level information to simulink in a binblock 
 * https://www.mathworks.com/help/instrument/binblockwrite.html
 * Brodcasts: | HEADER | '2' | '10' | output_sensor_1 ... output_sensor_4 output_Vcc (2 bytes *5 = 10 bytes)*/
void levelDataSend(){
  Serial.write(HEADER); // Send binblock header
  Serial.print("210"); // Send metadata (10 bytes sent)
  uint8_t * p; // Address water level data in bytes (Arduino adresses memory in Little Endian)
  p = (uint8_t *) &quadruple.v[0];
  for(uint8_t i = 0;i<sizeof(quadruple.v);i+=2){ // Send bytes in Big Endian
    Serial.write(*(p+i+1)); // Send most significant byte
    Serial.write(*(p+i)); // Send least significant byte
  }
}

// ---------- Configures 10 bit resolution for actuation pump inputs ----------
/* Configure digital pins 9 and 10 for 10-bit PWM output. */
void setupPWM10bit() {
    DDRB |= _BV(PB1) | _BV(PB2);        // Set D9 and D10 as outputs
    TCCR1A = _BV(COM1A1) | _BV(COM1B1)  // Set non-inverting PWM
        | _BV(WGM11);                   // Set mode 14: fast PWM, TOP=ICR1
    TCCR1B = _BV(WGM13) | _BV(WGM12)
        | _BV(CS11);                    // Set clock / 8 
    ICR1 = 0x03ff;                      // Set TOP counter value (10 bits)
}

/* 10-bit version of analogWrite() for pins D9 and D10. */
void analogWrite10bit(uint8_t pin, uint16_t val){
    switch (pin){
        case  9: OCR1A = val; break; // write pwm to D9
        case 10: OCR1B = val; break; // write pwm to D10
    }
}
