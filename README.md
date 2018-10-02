Sphero_SPRK
============
An python module that connects with Sphero SPRK+. Under development


Requirements:
--------------------
- Linux (bluepy & bluez dependence)
- \>= python3.3 

Installation
--------------------
To install the latest released version:

	$ pip install sphero_sprk
	$ cd <python site package>/bluepy
	$ make

Usage Example
---------------------------

	from sphero_sprk import Sphero	
	
	orb = Sphero("C8:A2:4D:7D:FA:4F")
	orb.connect()
	orb.set_rgb_led(255,0,0)


Currently supported commands
----------------------------------
 
 General
 - ping()
 - version()
 - get_device_name()


 Sphero
 - set_rgb_led(red, green, blue)
 - get_rgb_led()
 - set_stabilization(bool)
 - set_raw_motor_values(lmode, lpower, rmode, rpower)
 - set_heading(new_zero_heading_according_to_old_heading)
 - roll(heading, speed)

Listening for Streaming Messages
------------------------------------

Use "set_stream_callback" to your desired stream:
- set_stream_callback(self, name, callback, mask_id = 1):

Where name is one of the following:

On Mask 1:
- accel_raw
- gyro_raw
- emf_raw
- pwm_raw
- imu_filtered
- accel_filtered
- gyro_filtered
- emf_filtered

On Mask 2:
- quarternion
- odometer
- accelone
- velocity

And the callback is a method that will process a byte array of 16-bit ints

Specify which mask the item is on by using mask_id (2 for odometer for example)

 Common Errors
 ---------------------------------------
 
*  if program throws `FileNotFoundError: [Errno 2] No such file or directory: '/home/$USER/python3.4/site-packages/bluepy/bluepy-helper'.` Go to the directory where bluepy is installed(`/home/$USER/python3.4/site-packages/bluepy/`) and run the makefile located in the directory.
*  if the program halts at the beginning, restarting the program a few times will solve the problem. There's a known issue with bluepy sometimes getting stuck at the initialization phase.
*  if you get an error with the bluetooth, most likely you need to change the permissions of the bluetooth helper, try:
    
    setcap 'cap_net_raw,cap_net_admin+eip' \<Path to Python>/dist-packages/bluepy/bluepy-helper 

Contact & License
----------------------------------------------
Created and Maintained by CMU Assistive Robots Lab
Contact:  Zhi <zhi.tan@ri.cmu.edu>

Licensed under the MIT license
