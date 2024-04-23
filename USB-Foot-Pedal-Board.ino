/*
  Current PIN use:
  - 2       7x fastleds (WS2812B)
  - 3,4,5,6,7,8   footswitches

  - A0       ext FS  2
  - A1       ext FS  1

  - A3      (analog) INPUT R2    !RING (ANALOGREAD here)

  - 1       Serial Tx pin for Midi Out

  -14, 15   3-program selector switch

  Functions:

  Buttons
  - NoteOn/Off
  - Corresponding LED on/off on received NoteOn/Off

  INPUTS
  - Right input for external footswitch (soon: for expression pedal (potentiometer))
  - Left input for external footswitch
*/

////// LIBRARIES
#include <MIDIUSB.h>
#include <FastLED.h>
#include <ResponsiveAnalogRead.h>

// LED HUE codes
#define RED 0
#define GREEN 96
#define BLUE 160

//////////////////// !! VARIABLES !! ///
// !! MIDI SETTINGS !! //
const byte EXPRESSION_CC = 11; // Lowest MIDI CC to be used

///////////////
//// Buttons
const int NUM_BUTTONS = 8;                                       //*number of buttons (2 buttons + 2 encoder buttons + 1 digital crossfader)
const int BUTTON_PINS[NUM_BUTTONS] = {3, 4, 5, 6, 7, 8, A1, A0}; //* the number of the pushbutton pins in the desired order
const unsigned long DEBOUNCE_DELAY = 13;                         // the debounce time in ms; increase if the output flickers (default = 13)

int buttonCstate[NUM_BUTTONS] = {};               // stores the button current value
int buttonPstate[NUM_BUTTONS] = {};               // stores the button previous value
unsigned long lastDebounceTime[NUM_BUTTONS] = {}; // the last time the pin was toggled

// PROGRAMS
const byte PIN_PROG1 = 14;
const byte PIN_PROG2 = 15;

enum midiMessage
{
  Note,
  CC
};

// first 6 for F/S and last 2 for external F/S
const byte PROG1_VALUES[NUM_BUTTONS] = {36, 37, 38, 39, 40, 41, 42, 43};
const midiMessage PROG1_TYPES[NUM_BUTTONS] = {CC, CC, CC, CC, CC, CC, CC, CC};

const byte PROG2_VALUES[NUM_BUTTONS] = {24, 25, 26, 27, 28, 29, 30, 31};
const midiMessage PROG2_TYPES[NUM_BUTTONS] = {Note, Note, Note, Note, Note, Note, Note, Note};

const byte PROG3_VALUES[NUM_BUTTONS] = {0, 1, 2, 3, 4, 5, 6, 7};
const midiMessage PROG3_TYPES[NUM_BUTTONS] = {Note, Note, Note, Note, Note, Note, Note, Note};

const byte PROG1_COLOR = GREEN;
const byte PROG2_COLOR = RED;
const byte PROG3_COLOR = BLUE;

const byte PROG1_MIDI_CHANNEL = 1;
const byte PROG2_MIDI_CHANNEL = 9;
const byte PROG3_MIDI_CHANNEL = 9;

byte currentColor;
byte currentMidiChannel;
enum midiMessage currentMessageType[NUM_BUTTONS] = {};
byte currentProgram[NUM_BUTTONS] = {};

byte progPin1State;
byte progPin2State;

byte lastProgPin1State = 99;
byte lastProgPin2State = 99;

/////////////////////////////////////////////
// Potentiometers

const byte PIN_POTI = A3;
const int ANALOG_MAX = 1023;
const int ANALOG_MIN = 60;
const int NUM_POTS = 1;       //* number of potis
const int TIMEOUT = 800;      //* Amount of time the potentiometer will be read after it exceeds the varThreshold
const int VAR_THRESHOLD = 10; //* Threshold for the potentiometer signal variation

int potPin[NUM_POTS] = {PIN_POTI}; //* Pin where the potentiometer is
int potCState[NUM_POTS] = {};      // Current state of the pot
int potPState[NUM_POTS] = {};      // Previous state of the pot
int potVar = 0;                    // Difference between the current and previous state of the pot

int midiCState[NUM_POTS] = {}; // Current state of the midi value
int midiPState[NUM_POTS] = {}; // Previous state of the midi value

boolean potMoving = true;           // If the potentiometer is moving
unsigned long PTime[NUM_POTS] = {}; // Previously stored time
unsigned long timer[NUM_POTS] = {}; // Stores the time that has elapsed since the timer was reset

/////////////////
// rxMidi

byte rxUSB = 0;      // usb header
byte rxChannel = 0;  // midi channel
byte rxType = 0;     // midi data type
byte rxPitch = 0;    // midi pitch
byte rxVelocity = 0; // midi velocity

byte clockCounter;
bool ledOn = 0;

/////////////////////////////////////////////////
// LEDS

// WS2812B LEDS
#define NUM_LEDS 7 // Number of addressable LEDS
#define DATA_PIN 2 // LED PIN
#define LED_TYPE WS2812B
CRGB leds[NUM_LEDS];

////////////////////////////////////
/////////////////// !! SETUP !! ////
void setup()
{
  // Serial.begin(9600);   // turns on serial readout for debugging
  Serial1.begin(31250); // Set MIDI baud rate

  // program selector - set pullup resistor for 2 program selector pins
  pinMode(PIN_PROG1, INPUT_PULLUP);
  pinMode(PIN_PROG2, INPUT_PULLUP);

  // buttons
  for (int i = 0; i < NUM_BUTTONS; i++)
  {
    pinMode(BUTTON_PINS[i], INPUT_PULLUP); // sets pullup resistor mode for button pins
  }

  // poti
  pinMode(PIN_POTI, INPUT_PULLUP); // read expression pedal on this pin (RING)

  // addressable RGB LEDS
  FastLED.addLeds<LED_TYPE, DATA_PIN>(leds, NUM_LEDS);
  FastLED.clear();          // all addressable LEDs off during setup
  FastLED.setBrightness(8); // Brightness (0-255)
  FastLED.show();
}

/////////////////////////////////
//////////// !! LOOP !! /////////
void loop()
{
  checkProgram(); // check current program setting 1-3 & change midiout for buttons and expr.pedal
  buttons();      // 2xbutton input to midi out

  potentiometers(); // expression pedal

  refreshMidi(); // get latest received midi data
  rxMidiLeds();  // received Midi Notes to LED control (2xRGB)

  // serialDebug();
}

///////////////////////////////////
/////////////// FUNCTIONS /////////

// CHECKPROGRAM
void checkProgram()
{
  progPin1State = digitalRead(PIN_PROG1);
  progPin2State = digitalRead(PIN_PROG2);

  if (progPin1State != lastProgPin1State || progPin2State != lastProgPin2State)
  {

    // set to PROG1
    if (progPin1State == 1 && progPin2State == 0)
    {

      for (byte i = 0; i < NUM_BUTTONS; i = i + 1)
      {
        currentProgram[i] = PROG1_VALUES[i];
        currentMessageType[i] = PROG1_TYPES[i];
      }
      currentMidiChannel = PROG1_MIDI_CHANNEL;
      currentColor = PROG1_COLOR;
    }

    // set to PROG2
    if (progPin1State == 1 && progPin2State == 1)
    {

      for (byte i = 0; i < NUM_BUTTONS; i = i + 1)
      {
        currentProgram[i] = PROG2_VALUES[i];
        currentMessageType[i] = PROG2_TYPES[i];
      }
      currentMidiChannel = PROG2_MIDI_CHANNEL;
      currentColor = PROG2_COLOR;
    }

    // set to PROG3
    if (progPin1State == 0 && progPin2State == 1)
    {

      for (byte i = 0; i < NUM_BUTTONS; i = i + 1)
      {
        currentProgram[i] = PROG3_VALUES[i];
        currentMessageType[i] = PROG3_TYPES[i];
      }
      currentMidiChannel = PROG3_MIDI_CHANNEL;
      currentColor = PROG3_COLOR;
    }

    leds[0] = CHSV(currentColor, 255, 255); // change PROG indicator led colour depending on program
    FastLED.show();

    lastProgPin1State = progPin1State;
    lastProgPin2State = progPin2State;
  }
}
// BUTTONS
void buttons()
{

  for (int i = 0; i < NUM_BUTTONS; i++)
  {

    buttonCstate[i] = digitalRead(BUTTON_PINS[i]);

    if ((millis() - lastDebounceTime[i]) > DEBOUNCE_DELAY)
    {

      if (buttonPstate[i] != buttonCstate[i])
      {
        lastDebounceTime[i] = millis();

        if (buttonCstate[i] == LOW)
        {
          // ON
          if (currentMessageType[i] == Note)
          {
            noteOn(currentMidiChannel, currentProgram[i], 127); // channel, note, velocity}
          }
          else if (currentMessageType[i] = CC)
          {
            controlChange(currentMidiChannel, currentProgram[i], 127);
          }
          MidiUSB.flush();
        }
        else
        {
          // OFF
          if (currentMessageType[i] == Note)
          {
            noteOff(currentMidiChannel, currentProgram[i]); // channel, note, velocity
          }
          else if (currentMessageType[i] = CC)
          {
            controlChange(currentMidiChannel, currentProgram[i], 0);
          }
          MidiUSB.flush(); // send midi buffer (after each note)
        }
        buttonPstate[i] = buttonCstate[i];
      }
    }
  }
}

/////////////////////////////////////////////
// POTENTIOMETERS
void potentiometers()
{

  for (int i = 0; i < NUM_POTS; i++)
  { // Loops through all the potentiometers

    potCState[i] = analogRead(potPin[i]); // Reads the pot and stores it in the potCState variable
    Serial.println(potCState[i]);

    midiCState[i] = map(potCState[i], ANALOG_MAX, ANALOG_MAX, 0, 127); // Maps the reading of the potCState to a value usable in midi

    potVar = abs(potCState[i] - potPState[i]); // Calculates the absolute value between the difference between the current and previous state of the pot

    if (potVar > VAR_THRESHOLD)
    {                      // Opens the gate if the potentiometer variation is greater than the threshold
      PTime[i] = millis(); // Stores the previous time
    }

    timer[i] = millis() - PTime[i]; // Resets the timer 11000 - 11000 = 0ms

    if (timer[i] < TIMEOUT)
    { // If the timer is less than the maximum allowed time it means that the potentiometer is still moving
      potMoving = true;
    }
    else
    {
      potMoving = false;
    }

    if (potMoving == true)
    { // If the potentiometer is still moving, send the change control
      if (midiPState[i] != midiCState[i])
      {

        // use if using with ATmega328 (uno, mega, nano...)
        // MIDI.sendControlChange(EXPRESSION_CC+i, midiCState[i], currentMidiChannel);

        // use if using with ATmega32U4 (micro, pro micro, leonardo...)
        controlChange(currentMidiChannel, EXPRESSION_CC + i, midiCState[i]); // manda control change (channel, CC, value)
        MidiUSB.flush();

        // Serial.println(midiCState);
        potPState[i] = potCState[i]; // Stores the current reading of the potentiometer to compare with the next
        midiPState[i] = midiCState[i];
      }
    }
  }
}

/////////////////////////
///////////// midiLeds
void refreshMidi()
{
  // MidiUSB library commands
  midiEventPacket_t rx;
  rx = MidiUSB.read();

  // Bitshift to separate rx.byte1 (byte = 8bit) into first 4bit (TYPE), and second 4bit (Channel)
  rxChannel = rx.byte1 & B00001111; // get channel

  if (rxChannel == currentMidiChannel)
  {

    // Bitshift to separate rx.byte1 (byte = 8bit) into first 4bit (TYPE), and second 4bit (Channel)
    rxType = rx.byte1 >> 4;
    rxPitch = rx.byte2;    // Received pitch - midi-button-and-led first note setting
    rxVelocity = rx.byte3; // Velocity variable - can be used, for example, for brightness
    rxUSB = rx.header;     // get usb header
  }
}

void rxMidiLeds()
{
  // receive midi clock (type 15 / channels: 10=start; 12=stop; 8=pulse / 24 pulses per quarter note)

  // receive a start
  if (rxType == 15 && rxChannel == 10)
  {
    clockCounter = 0;
  }

  // receive clock pulse
  if (rxType == 15 && rxChannel == 8)
  {
    clockCounter++;
  }

  switch (clockCounter)
  {
  case 0:
    leds[0] = CHSV(currentColor, 255, 255); // change led colour depending on program
    FastLED.show();
    ledOn = 1;
    break;
  case 2:                    // length of each blink in midi pulses (default:2)
    leds[0] = CHSV(0, 0, 0); // change led colour depending on program
    FastLED.show();
    break;
  case 24: // mid signal transmits 24 pulses per quarter note
    clockCounter = 0;
    break;
  }

  // receive a stop -- turn led on permanently
  if (rxType == 15 && rxChannel == 12)
  {
    clockCounter = 0;
  }

  //////////////////
  // NOTEON received

  if (rxChannel == currentMidiChannel && rxType == 9)
  { // if receiving NoteON on correct channel

    // cycle through FASTLEDs
    for (int i = 1; i < NUM_LEDS; i++)
    { // cycle through all addressable LEDs - dont use LED 0!!

      if (rxType == 9 && rxPitch == currentProgram[i - 1] && currentMessageType[i - 1] == Note)
      { // if receiving noteON AND pitch matches LED
        int mapvelocity = map(rxVelocity, 0, 127, 0, 255);
        leds[i] = CHSV(mapvelocity, 255, 255); // control led by hue
        FastLED.show();
      }
    }
  }

  ///////////////////
  // NOTEOFF received

  if (rxChannel == currentMidiChannel && rxType == 8)
  { // if receiving NoteOFF on correct channel

    // cycle through FASTLEDs
    for (int i = 1; i < NUM_LEDS; i++)
    { // cycle through all addressable LEDs - dont use LED 0!!
      if (rxType == 8 && rxPitch == currentProgram[i - 1] && currentMessageType[i - 1] == Note)
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
    case 8:
      Serial.print("NoteOFF");
      break;
    case 9:
      Serial.print("NoteON");
      break;
    case 11:
      Serial.print("CC");
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

////////////////////////////////////////////////////////////////////////////////////////////
// Arduino (pro)micro midi functions MIDIUSB Library for sending CCs and noteON and noteOFF
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

  midiSerial(0xA0 | channel, control, value);
}

/////////////////////////////////////////////////////////////////////////////////////
// MIDI messages via serial bus

// Sends a midi signal on the serial bus
// cmd = message type and channel,
void midiSerial(byte cmd, byte pitch, byte velocity)
{
  Serial1.write(cmd);
  Serial1.write(pitch);
  Serial1.write(velocity);
}