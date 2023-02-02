# EnergyMeter
A digital watt-meter system has being designed. It measures the electric power used by a
single phase load in an electric circuit. This is a PIC microcontroler-based watt-meter prototype capable of reading voltage and current
waves. It uses the readings to calculate power-related data and then sends them to an LCD screen and to an ESP32 development board, which is in charge
of transmitting the data via WiFi. When accessing the ESP32’s local IP address in a LAN through a web browser, the user can see the real-time data displayed
in graphs.

It has only been tested using fictional loads. An RC circuit was used to simulate a phase shift.  By choosing R = 1kΩ and C = 47µF the theoretical maximum voltage
at the resistor would be VR = 498.9mV and, at the capacitor, 33.8mV.

This repository covers the ADC Concverter and the MCU module. The code for the wifi module can be found [here](https://github.com/rucoma00/EnergyMeter_WiFi_Com).

## Hardware specifications
This design will consider a 50Hz source frequency (Europe). The following picture shows the general schematic that the design follows. 
![General shematic](Media/Esquema_Basico_Vatimetro.png)


