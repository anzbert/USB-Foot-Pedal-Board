#include <MIDIUSB.h>
#include <FastLED.h>
#include <ResponsiveAnalogRead.h>

// NUMBER OF FOOT SWITCHES AND LEDS
const byte NUM_BUTTONS = 8; // 6 internal buttons + 2 external
const byte NUM_LEDS = 7;    // 0 is the indicator LED / 1-7 are the button LEDs

// PINS
// 'Serial1' Tx pin for DIN Midi Out = 1
const byte BUTTON_PINS[NUM_BUTTONS] = {3, 4, 5, 6, 7, 8, A1, A0}; // 6 foot switch pins + 2 external
const byte PIN_PROG1 = 14;                                        // 3-program selector switch PINs
const byte PIN_PROG2 = 15;                                        // 3-program selector switch PINs
const byte PIN_POTI = A3;                                         // (analog) EXPR. Pedal (Read on RING)
const byte LEDS_DATA_PIN = 2;                                     // 7x fastleds (WS2812B)

// LEDS ARRAY
CRGB leds[NUM_LEDS];

//// FOOT SWITCHES
const unsigned int DEBOUNCE_DELAY = 50; // debounce time in ms; increase if the output flickers

byte buttonPreviousState[NUM_BUTTONS] = {};       // stores the button previous value
unsigned long lastDebounceTime[NUM_BUTTONS] = {}; // the last time the pin was toggled

// MIDI MESSAGE TYPES
enum midiMessage
{
  NOTE,
  CC,
  PC, // 0-127 (equals Midi Programs 1-128)
  START,
  STOP,
  CONT
};

// ////////////////////////////////////////////////////////////////////////////////////
// ////////////////////////////////////////////////////////////////////////////////////
// PROGRAM SETTINGS
// ////////////////////////////////////////////////////////////////////////////////////

// Main Program Midi Channel in 0-15 (equals Midi Channels 1-16)
const byte CH1 = 0;
const byte CH2 = 8;
const byte CH3 = 8;

// Buttons: First 6 values for internal foot switches and last 2 for external

const byte PROG1_COLOR = HUE_GREEN;
const byte PROG1_VALUES[NUM_BUTTONS] = {88, 89, 91, 87, 0, 85, 100, 101};
const midiMessage PROG1_TYPES[NUM_BUTTONS] = {CC, CC, CC, CC, PC, CC, CC, CC};
const byte PROG1_CHANNELS[NUM_BUTTONS] = {CH1, CH1, CH1, CH1, CH1, CH1, CH1, CH1};

const byte PROG2_COLOR = HUE_RED;
const byte PROG2_VALUES[NUM_BUTTONS] = {24, 25, 26, 27, 28, 29, 30, 31};
const midiMessage PROG2_TYPES[NUM_BUTTONS] = {NOTE, NOTE, NOTE, NOTE, NOTE, NOTE, NOTE, NOTE};
const byte PROG2_CHANNELS[NUM_BUTTONS] = {CH2, CH2, CH2, CH2, CH2, CH2, CH2, CH2};

const byte PROG3_COLOR = HUE_BLUE;
const byte PROG3_VALUES[NUM_BUTTONS] = {0, 1, 2, 3, 4, 5, 6, 7};
const midiMessage PROG3_TYPES[NUM_BUTTONS] = {NOTE, NOTE, NOTE, NOTE, NOTE, NOTE, NOTE, NOTE};
const byte PROG3_CHANNELS[NUM_BUTTONS] = {CH3, CH3, CH3, CH3, CH3, CH3, CH3, CH3};

// EXPRESSION PEDAL CC
const byte EXPRESSION_CC = 11;

// ////////////////////////////////////////////////////////////////////////////////////
// ////////////////////////////////////////////////////////////////////////////////////

byte currentColor;
byte currentExprChannel;
enum midiMessage currentMessageType[NUM_BUTTONS] = {};
byte currentProgram[NUM_BUTTONS] = {};
byte currentChannel[NUM_BUTTONS] = {};

byte lastProgPin1State = 0xFF;
byte lastProgPin2State = 0xFF;

/////////////////////////////////////////////
// EXPRESSION PEDAL (Potentiometer)
const unsigned int ANALOG_MIN = 60;
const unsigned int ANALOG_MAX = 1023;
const unsigned int TIMEOUT = 800;      // Amount of time the potentiometer will be read after it exceeds the varThreshold
const unsigned int VAR_THRESHOLD = 10; // Threshold for the potentiometer signal variation

int potPreviousState;
unsigned long potPreviousTime;
boolean potStillMoving = true;
byte exprPreviousMidiValue;

/////////////////
// rxMidi

byte rxUSB = 0;      // usb header
byte rxChannel = 0;  // midi channel
byte rxType = 0;     // midi data type
byte rxPitch = 0;    // midi pitch
byte rxVelocity = 0; // midi velocity

byte midiClockCounter;

/////////////////////////////////////////////////////////////////////
/////////////////// !! SETUP !! /////////////////////////////////////
void setup()
{
  // IFNotDEFined, because code generates an error squiggle in VSCode with the
  // current arduino extension even if there is no problem with 'Serial1'
#ifndef __INTELLISENSE__
  Serial1.begin(31250); // Set MIDI baud rate
#endif
  // Serial.begin(9600);   // for debugging

  pinMode(PIN_PROG1, INPUT_PULLUP); // program selector pin1
  pinMode(PIN_PROG2, INPUT_PULLUP); // program selector pin2
  pinMode(PIN_POTI, INPUT_PULLUP);  // expr. pedal potentiometer pin (RING)
  for (int i = 0; i < NUM_BUTTONS; i++)
  {
    pinMode(BUTTON_PINS[i], INPUT_PULLUP); // foot switch pins
  }

  FastLED.addLeds<WS2812B, LEDS_DATA_PIN>(leds, NUM_LEDS); // init LEDs
  FastLED.setBrightness(8);                                // set Brightness (0-255)
  FastLED.clear();                                         // all LEDs off
  FastLED.show();                                          // refresh
}

//////////////////////////////////////////////////////////////////
//////////// !! LOOP !! //////////////////////////////////////////
void loop()
{
  updateProgram();

  sendFootSwitchMidi();

  sendExpressionPedalMidi();

  receiveMidi();

  updateLeds();

  // serialDebug();
}
//////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////

// CHECKPROGRAM
void updateProgram()
{
  byte progPin1State = digitalRead(PIN_PROG1);
  byte progPin2State = digitalRead(PIN_PROG2);

  if (progPin1State != lastProgPin1State || progPin2State != lastProgPin2State)
  {

    // set to PROG1
    if (progPin1State == 1 && progPin2State == 0)
    {

      for (byte i = 0; i < NUM_BUTTONS; i = i + 1)
      {
        currentProgram[i] = PROG1_VALUES[i];
        currentMessageType[i] = PROG1_TYPES[i];
        currentChannel[i] = PROG1_CHANNELS[i];
      }
      currentExprChannel = CH1;
      currentColor = PROG1_COLOR;
    }

    // set to PROG2
    if (progPin1State == 1 && progPin2State == 1)
    {

      for (byte i = 0; i < NUM_BUTTONS; i = i + 1)
      {
        currentProgram[i] = PROG2_VALUES[i];
        currentMessageType[i] = PROG2_TYPES[i];
        currentChannel[i] = PROG2_CHANNELS[i];
      }
      currentExprChannel = CH2;
      currentColor = PROG2_COLOR;
    }

    // set to PROG3
    if (progPin1State == 0 && progPin2State == 1)
    {

      for (byte i = 0; i < NUM_BUTTONS; i = i + 1)
      {
        currentProgram[i] = PROG3_VALUES[i];
        currentMessageType[i] = PROG3_TYPES[i];
        currentChannel[i] = PROG3_CHANNELS[i];
      }
      currentExprChannel = CH3;
      currentColor = PROG3_COLOR;
    }

    leds[0] = CHSV(currentColor, 255, 255); // change PROG indicator led colour depending on program
    FastLED.show();

    lastProgPin1State = progPin1State;
    lastProgPin2State = progPin2State;
  }
}
// BUTTONS
void sendFootSwitchMidi()
{
  for (int i = 0; i < NUM_BUTTONS; i++)
  {
    if ((millis() - lastDebounceTime[i]) > DEBOUNCE_DELAY)
    {
      byte buttonCurrentState = digitalRead(BUTTON_PINS[i]);

      if (buttonPreviousState[i] != buttonCurrentState)
      {
        lastDebounceTime[i] = millis();

        if (buttonCurrentState == LOW)
        {
          // ON
          if (currentMessageType[i] == midiMessage::NOTE)
          {
            noteOn(currentChannel[i], currentProgram[i], 127); // channel, note, velocity}
          }
          else if (currentMessageType[i] == midiMessage::CC)
          {
            controlChange(currentChannel[i], currentProgram[i], 127);
          }
          else if (currentMessageType[i] == midiMessage::PC)
          {
            programChange(currentChannel[i], currentProgram[i]);
          }
          else if (currentMessageType[i] == midiMessage::START)
          {
            midiStart(currentChannel[i]);
          }
          else if (currentMessageType[i] == midiMessage::STOP)
          {
            midiStop(currentChannel[i]);
          }
          else if (currentMessageType[i] == midiMessage::CONT)
          {
            midiCont(currentChannel[i]);
          }
        }
        else
        {
          // OFF
          if (currentMessageType[i] == midiMessage::NOTE)
          {
            noteOff(currentChannel[i], currentProgram[i]); // channel, note, velocity
          }
          else if (currentMessageType[i] == midiMessage::CC)
          {
            controlChange(currentChannel[i], currentProgram[i], 0);
          }
        }
        MidiUSB.flush(); // send midi buffer

        buttonPreviousState[i] = buttonCurrentState;
      }
    }
  }
}

/////////////////////////////////////////////
// EXPRESSION PEDAL
void sendExpressionPedalMidi()
{
  int potCurrentState = analogRead(PIN_POTI); // Reads the pot and stores it in the potCurrentState variable
  // Serial.println(potCurrentState);

  byte exprCurrentMidiValue = map(potCurrentState, ANALOG_MIN, ANALOG_MAX, 0, 127); // Maps the reading of the potCurrentState to a value usable in midi

  int potChange = abs(potCurrentState - potPreviousState); // Calculates the absolute value between the difference between the current and previous state of the pot

  if (potChange > VAR_THRESHOLD)
  {
    // Opens the gate if the potentiometer variation is greater than the threshold
    potPreviousTime = millis(); // Stores the previous time
  }

  if (millis() - potPreviousTime < TIMEOUT)
  {
    // If the potTimer is less than the maximum allowed time it means that the potentiometer is still moving
    potStillMoving = true;
  }
  else
  {
    potStillMoving = false;
  }

  if (potStillMoving)
  {
    // If the potentiometer is still moving, send the change control
    if (exprPreviousMidiValue != exprCurrentMidiValue)
    {
      controlChange(currentExprChannel, EXPRESSION_CC, exprCurrentMidiValue);
      MidiUSB.flush();

      // Serial.println(exprCurrentMidiValue);
      potPreviousState = potCurrentState; // Stores the current reading of the potentiometer to compare with the next
      exprPreviousMidiValue = exprCurrentMidiValue;
    }
  }
}

/////////////////////////
///////////// midiLeds
void receiveMidi()
{
  // MidiUSB library commands
  midiEventPacket_t rx;
  rx = MidiUSB.read();

  // Bitshift to separate rx.byte1 (byte = 8bit) into first 4bit (TYPE), and second 4bit (Channel)
  rxChannel = rx.byte1 & B00001111; // get channel

  // Bitshift to separate rx.byte1 (byte = 8bit) into first 4bit (TYPE), and second 4bit (Channel)
  rxType = rx.byte1 >> 4;
  rxPitch = rx.byte2;    // Received pitch - midi-button-and-led first note setting
  rxVelocity = rx.byte3; // Velocity variable - can be used, for example, for brightness
  rxUSB = rx.header;     // get usb header
}

void updateLeds()
{
  //////////////////////////
  // CLOCK

  // receive midi clock (type 0xF / channels: 0xA=start; 0xC=stop; 0x8=pulse / 24 pulses per quarter note)

  // receive a start
  if (rxType == 0xF && rxChannel == 0xA)
  {
    midiClockCounter = 0;
  }

  // receive clock pulse
  if (rxType == 0xF && rxChannel == 0x8)
  {
    midiClockCounter++;
  }

  switch (midiClockCounter)
  {
  case 0:
    leds[0] = CHSV(currentColor, 255, 255); // change led colour depending on program
    FastLED.show();
    break;
  case 2:                    // length of each blink in midi pulses (default:2)
    leds[0] = CHSV(0, 0, 0); // change led colour depending on program
    FastLED.show();
    break;
  case 24: // mid signal transmits 24 pulses per quarter note
    midiClockCounter = 0;
    break;
  }

  // receive a stop -- turn led on permanently
  if (rxType == 0xF && rxChannel == 0xC)
  {
    midiClockCounter = 0;
  }

  ////////////////////////////////////////
  // ALL CHANNEL SPECIFIC MIDI RX BELOW THIS GUARD

  bool matchAnyChannel = false;

  if (rxChannel == currentExprChannel)
    matchAnyChannel = true;

  for (byte i = 0; i < NUM_BUTTONS; i = i + 1)
  {
    if (rxChannel == currentChannel[i])
      matchAnyChannel = true;
  }

  if (!matchAnyChannel)
    return;

  //////////////////
  // NOTEON or CC received
  if (rxType == 0x9 || rxType == 0xB)
  {
    enum midiMessage receivedEnumType = rxType == 0x9 ? midiMessage::NOTE : midiMessage::CC;

    // cycle through FASTLEDs
    for (int i = 1; i < NUM_LEDS; i++)
    { // cycle through all addressable LEDs - dont use LED 0!!

      if (rxPitch == currentProgram[i - 1] && currentMessageType[i - 1] == receivedEnumType && rxChannel == currentChannel[i - 1])
      { // if receiving noteON AND pitch matches LED
        int mapvelocity = map(rxVelocity, 0, 127, 0, 255);
        leds[i] = CHSV(mapvelocity, 255, 255); // control led by hue
        FastLED.show();
      }
    }
  }

  ///////////////////
  // NOTEOFF received
  if (rxType == 0x8)
  {
    // cycle through FASTLEDs
    for (int i = 1; i < NUM_LEDS; i++)
    { // cycle through all addressable LEDs - dont use LED 0!!
      if (rxPitch == currentProgram[i - 1] && currentMessageType[i - 1] == midiMessage::NOTE && rxChannel == currentChannel[i - 1])
      {                          // if receiving noteOFF AND pitch  matches LED
        leds[i] = CHSV(0, 0, 0); // turn LED off
        FastLED.show();
      }
    }
  }
}

void serialDebug()
{
  // DEBUG raw rxMidi data
  if (rxUSB != 0)
  {
    Serial.print("USB-Header: ");
    Serial.print(rxUSB, HEX);

    Serial.print(" / Type: ");
    switch (rxType)
    {
    case 0x08:
      Serial.print("NoteOFF");
      break;
    case 0x09:
      Serial.print("NoteON");
      break;
    case 0x0B:
      Serial.print("CC");
      break;
    case 0x0C:
      Serial.print("PC");
      break;
    default:
      Serial.print("[");
      Serial.print(rxType);
      Serial.print("]");
      break;
    }

    Serial.print(" / Channel: ");
    Serial.print(rxChannel);

    Serial.print(" / Pitch: ");
    Serial.print(rxPitch);

    Serial.print(" / Velocity: ");
    Serial.println(rxVelocity);
  }
}

/////////////////////////////////////////////////////////////////////////////////////
// MIDI messages via serial bus

// Sends a midi signal on the serial bus
// cmd = message type and channel,
// 0xFF is out of the 7bit midi range and will not be sent
void midiSerial(byte cmd, byte pitch = 0xFF, byte velocity = 0xFF)
{
#ifndef __INTELLISENSE__
  Serial1.write(cmd);
#endif
  if (pitch <= 0x7F)
  {
#ifndef __INTELLISENSE__
    Serial1.write(pitch);
#endif
  }
  if (velocity <= 0x7F)
  {
#ifndef __INTELLISENSE__
    Serial1.write(velocity);
#endif
  }
}

//////////////////////////////////////////////////////////////////////////////
// Arduino (pro)micro midi functions MIDIUSB Library for sending Midi Messages

void midiStart(byte channel)
{
  midiEventPacket_t ccPacket = {0x0F, 0xFA | channel, 0x00, 0x00};
  MidiUSB.sendMIDI(ccPacket);

  midiSerial(0xFA | channel);
}

void midiStop(byte channel)
{
  midiEventPacket_t ccPacket = {0x0F, 0xFC | channel, 0x00, 0x00};
  MidiUSB.sendMIDI(ccPacket);

  midiSerial(0xFC | channel);
}

void midiCont(byte channel)
{
  midiEventPacket_t ccPacket = {0x0F, 0xFB | channel, 0x00, 0x00};
  MidiUSB.sendMIDI(ccPacket);

  midiSerial(0xFB | channel);
}

void noteOn(byte channel, byte pitch, byte velocity)
{
  midiEventPacket_t noteOnPacket = {0x09, 0x90 | channel, pitch, velocity};
  MidiUSB.sendMIDI(noteOnPacket);

  midiSerial(0x90 | channel, pitch, velocity);
}

void noteOff(byte channel, byte pitch)
{
  // midiEventPacket_t noteOffPacket = {0x08, 0x80 | channel, pitch, 0}; // note off message
  midiEventPacket_t noteOnPacket = {0x09, 0x90 | channel, pitch, 0}; // note on with velocity 0
  MidiUSB.sendMIDI(noteOnPacket);

  // midiSerial(0x80 | channel, pitch, 0); // note off message
  midiSerial(0x90 | channel, pitch, 0); // note on with velocity 0
}

void controlChange(byte channel, byte control, byte value)
{
  midiEventPacket_t ccPacket = {0x0B, 0xB0 | channel, control, value};
  MidiUSB.sendMIDI(ccPacket);

  midiSerial(0xB0 | channel, control, value);
}

void programChange(byte channel, byte program)
{
  midiEventPacket_t ccPacket = {0x0C, 0xC0 | channel, program, 0x00};
  MidiUSB.sendMIDI(ccPacket);

  midiSerial(0xC0 | channel, program);
}
