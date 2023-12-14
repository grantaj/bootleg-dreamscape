#include "MIDIUSB.h"
#include <NewPing.h>
#include "GP2Y0A02YK0F.h"
#include <movingAvg.h>
#include <DueTimer.h>

// ----
// Wrapper class for sensors
// providing moving average and nornalisation methods

class Sensor {
public:
  Sensor(unsigned long (*f)(), unsigned long low, unsigned long high, int filterLength)
    : sensorFunction(f), filter(filterLength), rangeLow(low), rangeHigh(high), lastValue(0) {
    filter.begin();
  }

  int read() {
    int sensorValue = sensorFunction();
    if (sensorValue > rangeLow && sensorValue <= rangeHigh) {
      filter.reading(sensorValue);
      return sensorValue;
    }

    return rangeLow;
  }

  int value() {
    return lastValue = filter.getAvg();
  }

  uint16_t normalisedValue() {
    lastValue = filter.getAvg();
    unsigned long c = min(max(rangeLow, lastValue), rangeHigh);
    return (((uint16_t)-1) * (c - rangeLow)) / (rangeHigh - rangeLow);
  }

  bool updated() {
    return filter.getAvg() != lastValue;
  }

private:

  unsigned long (*sensorFunction)();
  movingAvg filter;
  unsigned long rangeLow;
  unsigned long rangeHigh;
  unsigned long lastValue;
};

// MIDI ------------------------------------------------------------------------------
// Note on & Note off
// Argument is a struct with members
// header - event (0x09 = note on, 0x08 = note off, 0x0B = control change).
// byte 1 - event bitwise OR channel (0-15) channel is reported to the user as 1-16.
// byte 2 - note number (48 = middle C) or control channel (0-119)
// byte 3 - velocity (64 = normal, 127 = fastest) or control value (0-127)

#define MIDICC 0x0B
#define MIDIMAXCC 127
#define MIDIMAXPITCH 16383
#define REPORTDELAY 500


void noteOn(byte channel, byte pitch, byte velocity) {
  midiEventPacket_t noteOn = { 0x09, 0x90 | channel, pitch, velocity };
  MidiUSB.sendMIDI(noteOn);
}

void noteOff(byte channel, byte pitch, byte velocity) {
  midiEventPacket_t noteOff = { 0x08, 0x80 | channel, pitch, velocity };
  MidiUSB.sendMIDI(noteOff);
}

void controlChange(byte channel, byte control, byte value) {
  midiEventPacket_t event = { 0x0B, 0xB0 | channel, control, value };
  MidiUSB.sendMIDI(event);
}

void pitchChange(byte channel, uint16_t value) {
  byte low = value & 127;
  byte high = (value >> 7) & 127;

  midiEventPacket_t event = { 0x0E, 0xE0 | channel, low, high };
  MidiUSB.sendMIDI(event);
}

// SR1230 Rotary encoder  -------------------------------------------------------------
#define enc_dt 2
#define enc_clk 3
#define enc_button 4
volatile int encoderValue = 8192;
volatile int encoderButton = 1;

void setupRotaryEncoder(void) {
  pinMode(enc_dt, INPUT_PULLUP);
  pinMode(enc_clk, INPUT_PULLUP);
  pinMode(enc_button, INPUT);

  attachInterrupt(digitalPinToInterrupt(enc_clk), encoder, FALLING);
  attachInterrupt(digitalPinToInterrupt(enc_button), button, CHANGE);
}

void button() {
  encoderButton = digitalRead(enc_button);
  Serial.println("button!");
}


void encoder() {
  if (digitalRead(enc_clk) == digitalRead(enc_dt)) {
    encoderValue++;
  } else {
    encoderValue--;
  }

  encoderValue = min(16383, encoderValue);
  encoderValue = max(0, encoderValue);
}
// HC-SR04 Ultrasonic sensor ---------------------------------------------------------
#define TRIGGER_PIN 7
#define ECHO_PIN 6
#undef TRIGGER_WIDTH
#define TRIGGER_WIDTH 10
#define MIN_DISTANCE 0
#define MAX_DISTANCE 100  // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
#define FILTER_LENGTH 5

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);  // NewPing setup of pins and maximum distance.

unsigned long readSonar() {
  return sonar.ping_cm();
}

Sensor sonarSensor(readSonar, MIN_DISTANCE, MAX_DISTANCE, FILTER_LENGTH);

// ARD2-3005 Light sensor  ------------------------------------------------------------------------------
#define LIGHTSENSOR A0
#define MAX_LIGHT 1023

float readLightSensor() {
  return analogRead(LIGHTSENSOR);
}

// Thin film pressure sensor
#define PRESSURESENSOR A1
#define MAX_PRESSURE 1023

unsigned long readPressureSensor() {
  return analogRead(PRESSURESENSOR);
}

Sensor pressureSensor(readPressureSensor, 0, 1023, 5);

// Infrared Distance Sensor  ------------------------------------------------------------------------------
#define IRSENSOR A2
#define model 20150

GP2Y0A02YK0F ir(IRSENSOR);

unsigned long readInfraredSensor() {
  return ir.Ranging(_5V, CM);
}

Sensor infraredSensor(readInfraredSensor, 20, 50, 5);

// Generic parameter conditioning for midi cc  ------------------------------------------------------------------------------
float normalise(float x, float max) {
  return x / max;
}

int normaliseToMIDICC(float x, float max) {
  return MIDIMAXCC * clip(normalise(x, max));
}

int normaliseToMIDIPitch(float x, float max) {
  return MIDIMAXPITCH * clip(normalise(x, max));
}

float clip(float x) {
  return max(0, min(1, x));
}

void setup() {
  Serial.begin(115200);

  // timer interval is in microseconds
  Timer3.attachInterrupt(readHandler);
  Timer3.start(50e3);
  Timer4.attachInterrupt(writeHandler);
  Timer4.start(100e3);

  //setupRotaryEncoder();
}


Sensor pitchSensor(sonarSensor);
Sensor modWheelSensor(infraredSensor);

volatile bool executeRead = false;
volatile bool executeWrite = false;

void readHandler() {
  executeRead = true;
}

void writeHandler() {
  executeWrite = true;
}

void loop() {

  if (executeRead) {
    pitchSensor.read();
    modWheelSensor.read();
    executeRead = false;
  }

  if (executeWrite) {
    if (pitchSensor.updated()) {
      uint16_t pitchValue = pitchSensor.normalisedValue() >> 2;
      pitchValue = 16383 - pitchValue;
      //Serial.print("pitch:\t");
      //Serial.println(pitchSensor.value());
      pitchChange(1, pitchValue);
    }

    if (modWheelSensor.updated()) {
      byte modWheelValue = modWheelSensor.normalisedValue() >> 9;
      modWheelValue = 127 - modWheelValue;
      // Serial.print("mod:\t");
      // Serial.println(modWheelSensor.value());
      controlChange(1, 1, modWheelValue);
    }
    executeWrite = false;
  }
}