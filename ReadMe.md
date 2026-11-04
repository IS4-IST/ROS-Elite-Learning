Read the IS⁴ policies and guidelines to understand how you can contribute to your project in this repository, 
what structure is expected, and how to cultivate an outstanding project that will leave a lasting impact.

**Let’s build the best research lab since Marie Curie’s!**

The project currently only has:

1) the source code required to launch the camera communication. If you want to check it out, go to the instructions found in tests/camera/;

2) the robot communication pipeline.

3) a joint program that broadcasts both images and robot data as two (or more, depending on the number of cameras) ROS publishers.

## Instructions for Setup

# Camera related setup

You will have to install the express idf sdk

# Robot related setup

You will have to install the robot sdk. 

# ROS related setup

You will have to install the ROS2 sdk. 


is4ist@is4ist-desktop:~$ nmcli -f SSID,CHAN,SIGNAL dev wifi
SSID           CHAN  SIGNAL 
esp32cam       6     100    
eduroam        13    100    
tecnico-guest  13    100    
TeamQuad       1     99     
tecnico-guest  64    97     
eduroam        64    90     
--             4     82     
labofi         4     82     
--             1     75     
VIGILANT       1     75     
is4ist@is4ist-desktop:~$ iw dev
phy#0
	Interface wlxe4fac428a57c
		ifindex 3
		wdev 0x1
		addr e4:fa:c4:28:a5:7c
		type managed
		channel 6 (2437 MHz), width: 40 MHz, center1: 2447 MHz
		txpower 20.00 dBm
		multicast TXQ:
			qsz-byt	qsz-pkt	flows	drops	marks	overlmt	hashcoltx-bytes	tx-packets
			0	0	0	0	0	0	0	00
is4ist@is4ist-desktop:~$ iw dev wlxe4fac428a57c get power_save
Power save: off
is4ist@is4ist-desktop:~$ iw dev wlxe4fac428a57c link
Connected to 70:4b:ca:16:a8:a9 (on wlxe4fac428a57c)
	SSID: esp32cam
	freq: 2437
	RX: 107697672 bytes (88889 packets)
	TX: 196596 bytes (5819 packets)
	signal: -33 dBm
	rx bitrate: 135.0 MBit/s MCS 7 40MHz
	tx bitrate: 150.0 MBit/s MCS 7 40MHz short GI

	bss flags:	short-preamble short-slot-time
	dtim period:	1
	beacon int:	100
is4ist@is4ist-desktop:~$ 

