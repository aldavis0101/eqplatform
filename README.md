# eqplatform

This is an Arduino sketch to control the electronics for an equatorial tracking platform for a telescope.

Details of the whole platform project are in a blogspot page called [eqplatform2023](https://eqplatform2023.blogspot.com/). 

The Arduino circuit is on my google drive: [Circuit Diagram](https://drive.google.com/file/d/1gicNwj2TUNKHIdfqD5aNTAufZA7o-dyX/view?usp=sharing)

All the calculations for the motor's speed control are on the 'Drive Motor' tab of this [spreadsheet](https://docs.google.com/spreadsheets/d/1HPHdjHrK4HbHty3gnDlWtiA2XRQpFr5X75xHu2W92LE/edit?usp=sharing).

The electronic components for the project consist of:
- Arduino Nano v3
- TMC2209-based stepper driver module
- A NEMA 11 stepper motor
- A rotary encoder, for persistent speed control
- A joystick potentiometer, for momentary speed control
- A 128x64 OLED display

The sketch uses several libraries from the Arduino ecosystem:
- TMCStepper - to configure the TMC2209 stepper driver
- FastAccelStepper - to generate pulses to step the motor at the proper rate
- Encoder - to get the position of the encoder knob
- SoftwareSerial - to interface to the TMC2209 via UART
- SS_OLED - to control the display

Additional comments are in the code.
