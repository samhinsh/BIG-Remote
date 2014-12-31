BIG-Remote
==========
Author: Samuel Hinshelwood Jr. 

Functional Outline for the Bluetooth-Integrated Gyroscopic Remote Control

Software:
Sourcecode for a fully-integrated BIG Remote. Once connected, the Remote operates in 1 of 3 modes, MotionResponse, AutoRun, or ManualRun. Defaulted to ManualRun, the Remote will wait for input from the user via button-presses from a separate IR remote. Button presses, Operational Mode functions, and proper inputs are defined in the sourcode. Within each Operational Mode, the Remote will then interpret the user's input, and send "commands" accordingly to a Receiving Bluetooth Device in the form of ASCII characters. The Receiving Device will interpret these commands separately. The Interpretations for each input are defined in the sourcecode.

*Assumption: The HC-05 used in the circuit is already binded or paired with a Receiving Bluetooth Device

Generally, once initialized and connected to a separate Receiver, the Remote follows a sequence of actions:
Wait for user input (Ex:button press or hand motion) -> Interpret input -> Send command to Receiver

This software package handles all the initializations for each module in the circuit.

Hardware:
The circuit for the BIG Remote includes an ATMega328 Pro Mini Microcrontroller, a small breadboard (with the power rails removed for space conservation), an IR Receiver module, an mpu6050 gyroscope/acceleromter module, and an HC-05 Bluetooth module--in Master mode, pre-binded to a HC-05 module in Slave mode (can set using AT Commands and a Google search). The pinout for each module is described in the BIG Remote sourcode. An optional battery pack can be attached to the remote. 
