# Ernest Slave Node
Arduino sketch for data collection nodes

[Master Project Repo](https://github.com/rschlaikjer/Ernest)

## What is this
This sketch manages the data collection aspect of the Ernest system, gathering
temperature and pressure data then sending it to the master node over 2.4GHz.

This code is intended to be run on an arduino set up as laid out in the
[EAGLE schematic](https://github.com/rschlaikjer/ernest-slave-eagle), so if you
roll your own client board you may need to change
the pin numbers, which are defined at the top of the file.

## Debugging
Since these nodes are intended to run without easy connection to a serial
console, there is a basic error code set using the three LEDs. Here's what
each code means:
- All lights flashing on/off together every second: Panic. Could not init the
temp sensor.
- Lights turning on and off left to right, repeating: Idle.
- Solid blue: Writing data to radio
- Green + Blue: Data sent, awaiting ack
- Solid red: failed to write data to radio
- Red + Blue: No response from the base station
- Green: Transmit successful

## Early Prototype
![Early Prototype](/rev_0.1.jpg?raw=true "Early prototype")
