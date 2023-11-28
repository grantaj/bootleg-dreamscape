#include "MIDIUSB.h"

// HC-SR04 Ultrasonic sensor ---------------------------------------------------------
#define TRIG 7
#define ECHO 6
#define MAX_RANGE 100

// MIDI ------------------------------------------------------------------------------
// Note on & Note off
// Argument is a struct with members
// header - event (0x09 = note on, 0x08 = note off, 0x0B = control change).
// byte 1 - event bitwise OR channel (0-15) channel is reported to the user as 1-16.
// byte 2 - note number (48 = middle C) or control channel (0-119)
// byte 3 - velocity (64 = normal, 127 = fastest) or control value (0-127)

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

void setup() {
  Serial.begin(115200);
  usonicsetup();
}

void usonicsetup(void) {
  pinMode(ECHO, INPUT);
  pinMode(TRIG, OUTPUT);
  digitalWrite(TRIG, LOW);
}

float usonic() {

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

void loop() {

  float range = 0;
  int normalised_range = 0;
  long timeout = 1000;  // microseconds

  range = usonic();
  Serial.print("Range: ");
  Serial.print(range);
  Serial.print("\n");

  // MIDI CC is between 0 and 127
  normalised_range = 127*max(0,min(range/MAX_RANGE, 1)); 

  Serial.println("Sending control channel value");
  controlChange(0, 10, normalised_range);  // Set the value of controller 10 on channel 0 
  MidiUSB.flush();
  delay(500);
 
}