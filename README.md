# Smart-Bluetooth-Controller
Smart IoT Bluetooth sensors, scaling and integration gateway

# Smart Bluetooth Controller (Gateway) Asset tracking  
Smart IoT Bluetooth sensors, scaling and integration gateway

## Overview !

IoT applications that require scaling and multiple sensors several meters apart require the use of IoT gateways and integration with cloud apps and a local IoT Gateway aka controller. Bluetooth devices and beacons offer a variety of options and thus applications that allow scaling easily and cost-effectively. In this Smart IoT Bluetooth project I use different Bluetooth sensors in a system that can scale as needed. One application can be asset tracking using RSSI.

####  Features:
+ •	Measure valuable asset movement and send message if RSSI signal indicates it’s moving beyond a threshold with several asset tag sensors (Bluetooth sensors)
+•	Send notifications for RSSI if beacons move after a predefined value (Bluetooth sensors)
+•	Implement a Learning mode to register beacons to be tracked/monitored/observed
+•	Display (and refresh periodically) in a Dashboard the overall state of beacons: Total number of beacons, number of beacons that have gone into alarm state depending on a predefined value in dBm, number of beacons that have stopped to Advertise either because they’re far away, or they were stolen or the battery power drained
+•	Particle Argon for local controller
+•	OLED display
+•	GPS module to display current GPS location
+•	Button component to trigger learning (discovery) or normal operating mode.

Bluetooth devices broadcast services based upon their services. Usually asset locator BT devices broadcast every 6-8 seconds, but the time can be adjusted, and they don’t go into sleep mode. Therefore, the system will be constantly scanning for devices in the loop every 5 seconds to measure the RSSI and monitor the state of a beacon.
