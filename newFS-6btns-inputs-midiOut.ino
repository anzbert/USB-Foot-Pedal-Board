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

//////////////////// !! VARIABLES !! ///
// !! MIDI SETTINGS !! //
byte midiCh = -1 + 9; // MIDI channel to be used (1-16). default: 9
byte cc = 24;         // Lowest MIDI CC to be used

///////////////
//// Buttons
const int nButtons = 8;                                     //*number of buttons (2 buttons + 2 encoder buttons + 1 digital crossfader)
const int buttonPin[nButtons] = {3, 4, 5, 6, 7, 8, A1, A0}; //* the number of the pushbutton pins in the desired order
int buttonCstate[nButtons] = {};                            // stores the button current value
int buttonPstate[nButtons] = {};                            // stores the button previous value

unsigned long lastDebounceTime[nButtons] = {}; // the last time the pin was toggled
unsigned long debounceDelay = 13;              // the debounce time in ms; increase if the output flickers (default = 13)

// PROGRAMS
byte progPin1 = 14;
byte progPin2 = 15;

// first 6 for F/S and last 2 for external F/S
byte program1[nButtons] = {36, 37, 38, 39, 40, 41, 42, 43};
byte program2[nButtons] = {24, 25, 26, 27, 28, 29, 30, 31};
byte program3[nButtons] = {0, 1, 2, 3, 4, 5, 6, 7};

byte currentProgram[nButtons] = {};

byte progPin1State;
byte progPin2State;

byte lastProgPin1State = 99;
byte lastProgPin2State = 99;

//programLED HUE codes
byte red = 0;
byte green = 96;
byte blue = 160;
byte currentColor;

/////////////////////////////////////////////
// Potentiometers

byte pinPoti = A3;
int analogMax = 1023;
int analogMin = 60;

const int NPots = 1;           //* number of potis
int potPin[NPots] = {pinPoti}; //* Pin where the potentiometer is
int potCState[NPots] = {};     // Current state of the pot
int potPState[NPots] = {};     // Previous state of the pot
int potVar = 0;                // Difference between the current and previous state of the pot

int midiCState[NPots] = {}; // Current state of the midi value
int midiPState[NPots] = {}; // Previous state of the midi value

int TIMEOUT = 800;               //* Amount of time the potentiometer will be read after it exceeds the varThreshold
int varThreshold = 10;           //* Threshold for the potentiometer signal variation
boolean potMoving = true;        // If the potentiometer is moving
unsigned long PTime[NPots] = {}; // Previously stored time
unsigned long timer[NPots] = {}; // Stores the time that has elapsed since the timer was reset

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
  Serial.begin(9600); // turns on serial readout for debugging
  // Serial.begin(31250);  // Set MIDI baud rate:

  // program selector - set pullup resistor for 2 program selector pins
  pinMode(progPin1, INPUT_PULLUP);
  pinMode(progPin2, INPUT_PULLUP);

  // buttons
  for (int i = 0; i < nButtons; i++)
  {
    pinMode(buttonPin[i], INPUT_PULLUP); // sets pullup resistor mode for button pins
  }

  // poti
  pinMode(pinPoti, INPUT_PULLUP); // read expression pedal on this pin (RING)

  // addressable RGB LEDS
  FastLED.addLeds<LED_TYPE, DATA_PIN>(leds, NUM_LEDS);
  FastLED.clear();           // all addressable LEDs off during setup
  FastLED.setBrightness(10); // Brightness (0-255)
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
  progPin1State = digitalRead(progPin1);
  progPin2State = digitalRead(progPin2);

  if (progPin1State != lastProgPin1State || progPin2State != lastProgPin2State)
  {

    // set to program1
    if (progPin1State == 1 && progPin2State == 0)
    {

      for (byte i = 0; i < nButtons; i = i + 1)
      {
        currentProgram[i] = program1[i];
      }

      currentColor = green;
    }

    // set to program2
    if (progPin1State == 1 && progPin2State == 1)
    {

      for (byte i = 0; i < nButtons; i = i + 1)
      {
        currentProgram[i] = program2[i];
      }

      currentColor = red;
    }

    // set to program 3
    if (progPin1State == 0 && progPin2State == 1)
    {

      for (byte i = 0; i < nButtons; i = i + 1)
      {
        currentProgram[i] = program3[i];
      }

      currentColor = blue;
    }

    leds[0] = CHSV(currentColor, 255, 255); // change led colour depending on program
    FastLED.show();

    lastProgPin1State = progPin1State;
    lastProgPin2State = progPin2State;
  }
}
// BUTTONS
void buttons()
{

  for (int i = 0; i < nButtons; i++)
  {

    buttonCstate[i] = digitalRead(buttonPin[i]);

    if ((millis() - lastDebounceTime[i]) > debounceDelay)
    {

      if (buttonPstate[i] != buttonCstate[i])
      {
        lastDebounceTime[i] = millis();

        if (buttonCstate[i] == LOW)
        {

          noteOn(midiCh, currentProgram[i], 127); // channel, note, velocity
          MidiUSB.flush();
        }

        else
        {

          noteOn(midiCh, currentProgram[i], 0); // channel, note, velocity
          MidiUSB.flush();                      // send midi buffer (after each note)
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

  for (int i = 0; i < NPots; i++)
  { // Loops through all the potentiometers

    potCState[i] = analogRead(potPin[i]); // Reads the pot and stores it in the potCState variable
    Serial.println(potCState[i]);

    midiCState[i] = map(potCState[i], analogMin, analogMax, 0, 127); // Maps the reading of the potCState to a value usable in midi

    potVar = abs(potCState[i] - potPState[i]); // Calculates the absolute value between the difference between the current and previous state of the pot

    if (potVar > varThreshold)
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
        //MIDI.sendControlChange(cc+i, midiCState[i], midiCh);

        // use if using with ATmega32U4 (micro, pro micro, leonardo...)
        controlChange(midiCh, cc + i, midiCState[i]); // manda control change (channel, CC, value)
        MidiUSB.flush();

        //Serial.println(midiCState);
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

  if (rxChannel == midiCh)
  {

    // Bitshift to separate rx.byte1 (byte = 8bit) into first 4bit (TYPE), and second 4bit (Channel)
    rxType = rx.byte1 >> 4;
    rxPitch = rx.byte2;    // Received pitch - midi-button-and-led first note setting
    rxVelocity = rx.byte3; // Velocity variable - can be used, for example, for brightness
    rxUSB = rx.header;     //get usb header
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

  if (rxChannel == midiCh && rxType == 9)
  { // if receiving NoteON on correct channel

    // cycle through FASTLEDs
    for (int i = 1; i < NUM_LEDS; i++)
    { // cycle through all addressable LEDs - dont use LED 0!!

      if (rxType == 9 && rxPitch == currentProgram[i - 1])
      { // if receiving noteON AND pitch matches LED
        int mapvelocity = map(rxVelocity, 0, 127, 0, 255);
        leds[i] = CHSV(mapvelocity, 255, 255); // control led by hue
        FastLED.show();
      }
    }
  }

  ///////////////////
  // NOTEOFF received

  if (rxChannel == midiCh && rxType == 8)
  { // if receiving NoteOFF on correct channel

    // cycle through FASTLEDs
    for (int i = 1; i < NUM_LEDS; i++)
    { // cycle through all addressable LEDs - dont use LED 0!!
      if (rxType == 8 && rxPitch == currentProgram[i - 1])
      {                          // if receiving noteOFF AND pitch  matches LED
        leds[i] = CHSV(0, 0, 0); // turn LED off
        FastLED.show();
      }
    }
  }
}

void serialDebug()
{
  //DEBUG raw rxMidi data
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
  midiEventPacket_t noteOn = {0x09, 0x90 | channel, pitch, velocity};
  MidiUSB.sendMIDI(noteOn);
}

void noteOff(byte channel, byte pitch, byte velocity)
{
  midiEventPacket_t noteOff = {0x08, 0x80 | channel, pitch, velocity};
  MidiUSB.sendMIDI(noteOff);
}

void controlChange(byte channel, byte control, byte value)
{
  midiEventPacket_t event = {0x0B, 0xB0 | channel, control, value};
  MidiUSB.sendMIDI(event);
}

/////////////////////////////////////////////////////////////////////////////////////
// MIDI messages via serial bus

// Sends a midi signal on channel 1 (0x90)
// cmd = message type and channel,
void midinoteOn(int cmd, int pitch, int velocity)
{
  Serial.write(cmd);
  Serial.write(pitch);
  Serial.write(velocity);
}