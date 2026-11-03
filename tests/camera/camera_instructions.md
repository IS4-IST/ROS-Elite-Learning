First we need to install the express-idf toolkit on your computer. I only tested the following pipeline on linux, so forgive me if you have a different system, and good luck!

With the toolkit in your system, open a new terminal, and do 

```bach
cd {project directory}/tests/camera
{project directory}/tests/camera~ get_idf
```

This puts the toolkit in the path of the current terminal, so you are ready to go!
Connect yourself to the esp32-cam through USB. 

There are currently 4 projects in the camera that I used to build the comunication pipeline, namely 
1. TCPServer: this shows how to establish a server on the ESP32 using ASIO to communicate through WIFI with us. The ESP32 is currently in Access Point mode, so you need to connect to the board Wifi.

2. camerasnapshot: this shows how to use the camera API to retrieve snapshots. No message is actually sent to our computer. Think of this as a reference for us to use in the future

3. broadcaster: the broadcaster is an application that fuses both of the previous applications, so we communicate, and we also view the camera feed in our computer. 

4. broadcasterUDP:


```bach
{project directory}/tests/camera~ get_idf
```
