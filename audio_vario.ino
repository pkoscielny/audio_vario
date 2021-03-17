/*
 * IDE: Arduino IDE 1.8.13.                                                                                                                                                                                                                                                   
 * Board: Arduino Nano.                                                                                                                                                                                                                                            
 *  
 * Hardware:
 * - Arduino Nano.
 * - BMP280 pressure and temperature sensor.
 * - 3.3V <---> 5V logic converter + 2x 4.7k resistors.
 * - HC-06 Bluetooth module.
 * - buzzer.
 * - potentiometer (for buzzer) with integrated on-off switch.
 * - 2x push buttons + 2x 10k resistors.
 * - 500mA DC-DC boost converter step up 1-5V -> 5V.
 * - 2x Ni-MH 1.2V batteries.
 * 
 * generate_tone taken from https://github.com/jaffadog/arduino-variometer
 * It works perfectly.
*/

/* Hardware connection I2C:
  VCC to the 3V3 (important, do not use 5V, could destroy the sensor)
  GND to GND
  SDA to A4
  SCL to A5
*/

/* Hardware connection Bluetooth:
 VCC - red
 GND - black
 TXD - white
 RXD - yellow
*/

#include <Adafruit_Sensor.h>
#include "Adafruit_BMP280.h"
Adafruit_BMP280 bmp; // I2C - 0x76; SPI - 0x77.

// Buzzer
const int buzzer_pin = 3; // D3


void setup() {
  Serial.begin(9600);

  // Init sensor.
  bmp.begin();

  // Init buzzer.
  pinMode(buzzer_pin, OUTPUT);

  // Init buttons.
  pinMode(4, INPUT); // D4 + 10k resistor.
  pinMode(5, INPUT); // D5 + 10k resistor.
}


int i;
float pressure;      // to store the barometric pressure (Pa).
float pressure_diff; // to calibrate pressure by buttons.
float temperature;
void loop() {
  // Main time counter.
  static unsigned long int current_time = 0;
  current_time = micros();
    
  // Read values from the sensor:
  pressure = bmp.readPressure();

  // Handle pushing buttons - calibrate pressure to real QNH.
  handle_buttons(current_time);

  pressure -= pressure_diff;

  // Send LK8EX1.
  if (i++ > 10) {
    temperature = bmp.readTemperature();
    send_lk8ex1(pressure, (int)temperature);
    i = 0;
  }

  // Make sound.
  generate_tone(pressure);

  delay(20);
}



void generate_tone(float pressure) {
  static float tone_freq, tone_freq_lowpass,
               lowpass_fast = pressure, 
               lowpass_slow = pressure;
  static int dds_acc;

  lowpass_fast = lowpass_fast + (pressure - lowpass_fast) * 0.1;
  lowpass_slow = lowpass_slow + (pressure - lowpass_slow) * 0.05;
  tone_freq = (lowpass_slow - lowpass_fast) * 10; // tone increasing speed. Previously multiplier was 50.
  tone_freq_lowpass = tone_freq_lowpass + (tone_freq - tone_freq_lowpass) * 0.1;
  tone_freq = constrain(tone_freq_lowpass, -500, 500);
 
  dds_acc += tone_freq * 100 + 1000; // beep fequency. Previously was +2000.

  if (tone_freq < 0 || dds_acc > 0) {
    tone(buzzer_pin, tone_freq + 510);
  }
  else {
    noTone(buzzer_pin);
  }
  
}


// Send pressure and temperature to external device using LK8EX1 protocol.
// More info: https://github.com/LK8000/LK8000/blob/master/Docs/LK8EX1.txt
void send_lk8ex1(long pressure, int temperature) {
  String str_out = String("LK8EX1" 
    + String(",") + String(pressure, DEC) // raw pressure
    + String(",99999,9999,") // alt, vario - off
    + String(temperature, DEC) 
    + String(",999,") // battery voltage or charge percentage - off
  );
  
  // Calculating checksum for data string.
  unsigned int checksum_end, ai, bi;
  for (checksum_end = 0, ai = 0; ai < str_out.length(); ai++) {
  	bi = (unsigned char)str_out[ai];
    checksum_end ^= bi;
  }

  Serial.print("$");                  // print first sign of NMEA protocol
  Serial.print(str_out);              // print data string
  Serial.print("*");                  // end of protocol string
  Serial.println(checksum_end, HEX);  // print calculated checksum on the end of the string in HEX  
}


void handle_buttons(unsigned long int current_time) {
  static unsigned long int last_time;
  unsigned long int time_diff = current_time - last_time;

  // 5 x s
  if (time_diff < 20000) // was 200000
    return;
    
  // Button 'QNH up' pressed.
  if (digitalRead(4) == HIGH) {
    pressure_diff++;
  } 
  // Button 'QNH down' pressed.
  else if (digitalRead(5) == HIGH) {
    pressure_diff--;
  }
  
  // Save for next time.
  last_time = current_time;
}

