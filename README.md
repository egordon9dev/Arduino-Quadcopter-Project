# Quadcopter-Arduino-Flight-Controller

This quadcopter uses an accelerometer and a gyroscope combined by sensor fusion done on the MPU6050 to recieve the accurate angular position. This data is sent to the PID controller for stabilization. This repository contains the flight controller code that goes on the arduino on the quadcopter, and it also contains the remote controller code that runs on a seperate arduino. Both arduinos communicate with an nr24l01+ module. A high power one with the large antenna is used on the controller for transmitting signals and a lower power one is used on the quadcopter for recieving signals. 

[Check it out in action!](https://youtu.be/JmUGWP2XuXM?t=20)

<img src="https://github.com/egordon9dev/Arduino-Quadcopter-Project/blob/master/controller.jpg" width=250 />

<img src="https://github.com/egordon9dev/Arduino-Quadcopter-Project/blob/master/quadcopter.jpg" width=250 />
