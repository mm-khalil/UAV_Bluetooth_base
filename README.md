# UAV_Bluetooth_base
Build, Control and Fly your own Drone with your Android phone.


# OUR  DRONE:
	As we have a drone including chassis, motors, controller, ESC’s, propellers, and sensors like MPU 6050.
	Our task is to code it and fly it.
	So first we control motors by Potentiometer then with Bluetooth module by sending commands through android phone.
	We implement PID by configuring a sensor MPU 6050 by using its output as commands for controller.
	Now our future work is to control it by RC Controller.
	Follow this block:

## STEP # 1
### COMPONENTS  &  ASSEMBLY:
	4 Motors.
	Frame.
	4 Electronic Speed Controllers.
	Arduino UNO.
	MPU-6050 sensor.
	Bluetooth Module (HC 05).
	Battery (11v).
	RC Controller.
	Connecting Wires.
	Assemble all the things by following this tutorial and also this tutorial shows you the exact components which we are using except RC Controller:
https://www.mydronelab.com/blog/arduino-quadcopter.html

## STEP # 2
### CONNECTIONS:
	For 4 Motors and 4 ESC’s output:
ESC	Motors
Vcc-red	Vcc-red
GND-black	GND-black
Signal-yellow	Signal-yellow

	Inputs of each ESC connect with Arduino UNO.
#### ESC	Arduino UNO
1st  ESC PWM-white pin	3 pin
2nd  ESC PWM-white pin 	5 pin 
3rd  ESC PWM-white pin 	6 pin 
4th  ESC PWM-white pin 	9 pin 
GND  GND

#### 	Bluetooth     to    Arduino:
•	Keep in mind always connect Bluetooth module (Rx, Tx) to Arduino (Rx0, Tx0).
•	Make sure when you burn code to Arduino then you should every time remove from Rx0 Tx0 wires, when code uploaded then connect back to Rx0 Tx0.
•	Download official Arduino android app (Arduino Control) in your phone from play store, then power your drone and connect your android to HC 05 bluetooth, then open app and from app select terminal and send 1 or 0.
#### #Bluetooth	Arduino
Rx	Tx
Tx	Rx
Vcc	+5v
GND	GND


# YOUTUBE VIDEO LINK:
https://youtu.be/G1q5IrAnESs
