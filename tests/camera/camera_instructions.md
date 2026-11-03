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

3. broadcaster: the broadcaster is an application that fuses both of the previous applications, so we communicate, and we also view the camera feed in our computer. This requires us to have an end-point on our computer that can read the messages. On /tests/camera/python_viewer you can find viewers to connect to the remove ESP32 sockets that we use to communicate. The application has two modes to work, namely: 1) stream mode; 2) snapshot mode.  If you query for stream it will send a non-stop barrage of images at the maximim resolution that it can, while in snapshot mode, it will send a single isolated picture. 

4. broadcasterUDP: the broadcasterUDP has all the same features as the normal broadcaster, with the caveat that it uses UDP instead of TCP, so that frames are not delayed when packets are lost. 
