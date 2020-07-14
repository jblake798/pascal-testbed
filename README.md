# PascalQuad Sensor Testbed

These values will effect the timing:  
CPU Speed: 180MHz (DEFAULT IN VSCODE)  
Optimize: Fastest **(UNKNOWN HOW TO CHANGE THIS IN VSCODE)**  

## Main Description
sensors will update at the beginning of each loop iff they have data  
- i2c interrupts are checked -> if high, req new data (verify rate and time taken)  
- gps serial buffer is checked -> if something there, everything read (verify rate and time taken)  
every loop, imu quanternion filter will update  
every loop, kalman filter/system model will update  
every loop, updates will be sent to motors from controller  
keeping these last three tasks completing as fast as possibile is vital  
- ensures gps serial data is not kept waiting long  
- time the update parts of the loop and determine system update rate  
- determine how much of an impact sensor updates are (percentage of time reading sensors vs updating system)  
system update rate will be faster than sensor update rate  
- different than last quadcopter  
- will make the system model and kalman filter very important to get right  
sensor update rates will be kept reasonable, not super fast  
serial transmission offboard will be kept minimal  
- not every loop, but at predetermined rate (like 2Hz or something)  

## TODO  
- sd card recording - time how long reads and writes take  
- set up bluetooth  
- time an empty loop to verify processor speed  
- write Kalman filter class  
