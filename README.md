# JfsCounter
<p align="center">
   <img src="doku/jfspc.png" width="128">
</p>

## About
A pulse counter based on a ESP32 with a OLED display. It counts the nummer of pulses during a fixed time frame.The idea was to build a small and cheap counter as an interface between a device witch genenerates 5V TTL an a GUI that runs on a PC.


For more information please visit my [page](https://science.jefro.de). There is a windows installer available.

``` mermaid
flowchart LR
  id[TTL sending device]
  id1([ESP32 Counter])
  id2[PC Programm]
  id -->| pulses | id1 -->| >value< | id2
```
<p align="left">
   <img src="doku/esp32InAction.jpg" width="800">
</p>




## Software
### ESP32
The programmcode ** JFS_ESP32_Counter.ino ** is complide and uploaded via the ArduinoIDE:
<p align="left">
   <img src="doku/arduinoIDE.jpg" width="800">
</p>

##  PC Software Installation Guide using Virtual Environment
This guide explains how to install the JfsCounter PC software using Python Virtual Environment and requirements.txt.

### Prerequisites
Python 3.7 or higher installed

pip (usually installed with Python)

Git (optional, if cloning the repository)

### Installation Steps
- Download the project

   - Either clone the repository:

   -   git clone https://github.com/jfsScience/JfsCounter.git
   - cd JfsCounter
- Or download and extract the ZIP file

### Create a virtual environment

- python -m venv venv
### Activate the virtual environment

- Windows:

   - venv\Scripts\activate
- Linux/MacOS:

   - source venv/bin/activate
### Install dependencies

- pip install -r requirements.txt<br>

### Run the program

- python counter.py

## Hardware
### Finished Device
<p align="left">
   <img src="doku/IMG_7589_1024.jpg" width="512">
</p>

### Parts
-  ESP32-S WiFi Bluetooth Development Board 0,96 OLED Display WROOM 32 NodeMCU
-  Pegelwandler 4 Kanal I2C IIC Logic Level Converter 5V~3.3V 
-  BNC-connector | Board | IC socket

<p align="left">
   <img src="doku/IMG_7586_1024.jpg" width="512">
</p>

### Case
<p align="left">
   <img src="doku/IMG_7588_1024.jpg" width="512">
</p>

### Scheme
<p align="left">
   <img src="doku/Counterskizze.JPG" width="256">
</p>



## Thanks 

- This program is based the ESP32 Frequency Meter by Rui Viana and Gustavo Murta august/2020 [link](https://blog.eletrogate.com/esp32-frequencimetro-de-precisao)
- thanks to [pyshine](https://www.youtube.com/watch?v=JjtqLPbh9-o)

