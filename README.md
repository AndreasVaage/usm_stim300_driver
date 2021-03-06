
# STIM 300
https://www.sensonor.com/products/inertial-measurement-units/stim300/

## Dependensies

https://github.com/ros-drivers/rosserial

Run with:

    rosrun driver_stim300 stim300_driver_node

or to run the entire visual inertial sensor:

    roslaunch usm_stim300_driver usm_VI_sensor.launch
    
see launch file for available parameters.

## Comunicate with teensy

The teensy can be controlled by sending a ros message at the /VI_command topic. The first number, command, can be either 0 or 1 to turn it on or off, if 1 the second number, data, represents the number of imu messages between each image. The IMU rate is 125 Hz, the default is 7 imu messgages for each image, giving fps = 125/7 = 17.85.

    rostopic pub /VI_command usm_stim300_driver/UInt8UInt8 "command: 1 data: 7" -1

## Firmware

The firmaware code is designed for teensy 3.2 microcontroller using the arduino framework.
To program the microcontroller follow these steps:

* Set up arduino with the Teensyduino plugin, see https://www.pjrc.com/teensy/teensyduino.html.
* Build the rospackage ````catkin build --this ````
* Delete the old arduino ros_lib ```rm -r ~/Arduino/libraries/ros_lib```
* Create and export the ros_lib to your arduino library folder
    ```rosrun rosserial_arduino make_libraries.py ~/Arduino/libraries```
* Open firmware/stim300_driver/stim300_driver.ino in the arduino IDE and upload to the teensy microcontroller. 

## Communicate with STIM300 over terminal
For testing and configuration during development it can be useful to comunicate with the STIM300 IMU directly over terminal.
Here is a simple example for how to that using minicom.

Install minicom:

    sudo apt-get install minicom

Look for device:
    
    dmesg | grep tty

Open minicom with setings:

    sudo minicom -s

Setup minicom for stim300:

Serial port setup:

    A - Serial Device: /dev/ttyUSB0
    E - 921600 8N1
    F - Disable hardware flow control

Modem and dialing: (Clear option A...I)

    A -
    * -
    * -
    * -
    I -

Screen and keyboard:

    P - Add linefeed

Save setup as dfl then Exit minicom, and enter again in hex display mode:

    sudo minicom -H

The stim300 is in normal mode and will reapeatidly send the standard datagram.

Enter service mode: write "SERVICEMODE" and press enter

    SERVICEMODE

Clear the screen:

    Ctrl-A c

If the incomming datagram feed stopped it means you enter service mode sucsessfully. While in service mode the stim 300 will comunicate with asci characters. Exit minicom and enter in normal ASCI mode:

    Ctrl-A x
    sudo minicom

Write ? and press enter, and the STIM300 should send info about available commands

    ?

This will show a list on available commands including how to go back to normal mode. For example:

    c

 will perform a system check.
