# 6 Button USB Foot Pedal

## Background

This project was born out of the need for controlling music software and other hardware devices with a foot controller. Specifically, I wanted to use Ableton's loop effect while plaing the guitar. And to make sure that I would never run out of buttons, I went with the old saying of 'going big, or going home'. Not that I don't like going home 😛, but building it with expandability in mind seemed like a good idea, to cover for any future need. There are a few pedals available online, but they are typically priced quite high.

On top of USB communication, I wanted it to produce serial MIDI output for potentially linking the device with other music hardware. I also wanted it to be extendable with an expression pedal and with a secondary single or double footswitch.

## Solution

### Materials

- 1x Arduino Micro Pro (Leonardo)
- 2x 6.3mm Sockets
- 1x PCB
- 1x Midi DIN Socket
- 7x WS2811 RGB LEDS
- 6x Foot Switches (from AliExpress)
- 1x SPDT Switch
- 1x 220 Ohm Resistor
- 1x Panel Mount Micro USB Male to USB-B Female Adapter
- 1x Mountable Project box
- Wood, Cables, Cable Ties, Pipe

### Build

The build was accomplished with ordered parts plus a piece of wood and pipe from a trashpile outside the house. The hard wood plank is very heavy which funnily gives the pedal a high quality feel and prevents it from moving easily.

The Switches and external foot switch socket are wired to digital pins, and the external expression pedal socket is wired up like a potentiometer to an analog pin.

The LEDs are connected to a digital pin in series and the MIDI output port is connected to a serial Tx pin.

The program selector switch uses two digital pins and allows for the selection of 3 programs (ON 1 / OFF / ON 2).

### Code

- The software translates button presses from foot switches and the external switch and external expression pedal to MIDI signals. It then transmits notes and CC to the PC via USB and to external hardware via the DIN MIDI port.

- There are 3 programs selectable, which changes the MIDI output bank. The LED on the box indicates with RGB which program is currently selected.

- The LEDs above the switches show received MIDI signals from the PC, translated into RGB colors. This can, for example, show if a track is currently playing or recording.

## Conclusion

This pedal is a solid and versatile piece of equipment. Maybe building and coding was more fun than using it 😂. Ah well... it will come in handy one day I am sure. I just havn't gotten around to experimenting with it yet!

In any case, my build solution was significantly cheaper than what is availabe on the market, and I am happy with the result

## Links

[Source](https://github.com/anzbert/USB-Foot-Pedal-Board)
