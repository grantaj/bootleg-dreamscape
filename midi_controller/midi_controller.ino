#include "MIDIUSB.h"



// MIDI ------------------------------------------------------------------------------
// Note on & Note off
// Argument is a struct with members
// header - event (0x09 = note on, 0x08 = note off, 0x0B = control change).
// byte 1 - event bitwise OR channel (0-15) channel is reported to the user as 1-16.
// byte 2 - note number (48 = middle C) or control channel (0-119)
// byte 3 - velocity (64 = normal, 127 = fastest) or control value (0-127)

#define MIDICC 0x0B
#define MIDIMAXCC 127
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

// HC-SR04 Ultrasonic sensor ---------------------------------------------------------
#define TRIG 7
#define ECHO 6
#define MAX_RANGE 100

void setupUltrasonicSensor(void) {
  pinMode(ECHO, INPUT);
  pinMode(TRIG, OUTPUT);
  digitalWrite(TRIG, LOW);
}

float readUltrasonicSensor() {

  float duration;
  float distance;

  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);

  duration = pulseIn(ECHO, HIGH);
  distance = (duration * .0343) / 2;

  return distance;
}

// Light sensor
#define LIGHTSENSOR A0
#define MAX_LIGHT 1023

float readLightSensor() {
  return analogRead(LIGHTSENSOR);
}

float normalise(float x, float max) {
  return x/max;
}

int normaliseToMIDICC(float x, float max) {
  return MIDIMAXCC * clip(normalise(x, max));
}

float clip(float x) {
  return max(0,min(1,x));
}

void setup() {
  Serial.begin(115200);
  setupUltrasonicSensor();
}

void loop() {

  float range = 0;
  float light = 0;

  range = readUltrasonicSensor();
  light = readLightSensor();

  controlChange(0, 1, normaliseToMIDICC(range, MAX_RANGE));
  MidiUSB.flush();
  controlChange(0, 2, normaliseToMIDICC(light, MAX_LIGHT));  
  MidiUSB.flush();
  delay(REPORTDELAY);
}