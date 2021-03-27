// This program is used to send streaming accelerometer data from MMA8451 through Bluefruit LE using Teensy 4.0 //

// Headers for Teensy & Accelerometer //
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MMA8451.h>
#include <Adafruit_Sensor.h>
// Headers for Bluetooth //

#define xbee Serial1

// Accelerator Class //
Adafruit_MMA8451 mma = Adafruit_MMA8451();
// Samplerate for accelerometer //
int data_samplerate = 5; // Hz
int data_delay = 1000/data_samplerate;
#define MMA8451_SAMPLERATE_DELAY_MS (data_delay)

void setup(void)
{
  Serial.begin(9600); // Serial connection to USB
  xbee.begin(115200); // Serial connection to XBEE
  // while (!Serial);
  Serial.println("Adafruit MMA8451 test!");

 
  // Start accelerometer
  if (! mma.begin()){
    Serial.println("Couldnt start");
  }
  Serial.println("MMA8451 found!");

  // Set G-range for accelerometer //
  mma.setRange(MMA8451_RANGE_8_G);
  Serial.print("Range = "); Serial.print(2 << mma.getRange());
  Serial.println("G");
  Serial.println(F("Done!"));
}

void loop(void)
  {
  // Read the 'raw' data in 14-bit counts
  // mma.read();

  // /* Get a new sensor event */
  sensors_event_t event;
  mma.getEvent(&event);

  // /* Display the results (acceleration is measured in m/s^2) */
  Serial.print(event.acceleration.x,4);
  Serial.print("\t");
  Serial.print(event.acceleration.y,4);
  Serial.print("\t");
  Serial.println(event.acceleration.z,4);

  xbee.print(event.acceleration.x,4);
  xbee.print("\t");
  xbee.print(event.acceleration.y,4);
  xbee.print("\t");
  xbee.println(event.acceleration.z,4);


  delay(MMA8451_SAMPLERATE_DELAY_MS);
}
