# PascalQuad Sensor Testbed

These values will effect the timing:  
CPU Speed: 180MHz (DEFAULT IN VSCODE)  
Optimize: Fastest **(UNKNOWN HOW TO CHANGE THIS IN VSCODE)**  

## Main Description
1. sensors will update at the beginning of each loop iff they have data  
  1. i2c interrupts are checked -> if high, req new data (verify rate and time taken)  
  2. gps serial buffer is checked -> if something there, everything read (verify rate and time taken)  
2. every loop, imu quanternion filter will update  
3. every loop, kalman filter/system model will update  
4. every loop, updates will be sent to motors from controller  
5. keeping these last three tasks completing as fast as possibile is vital  
  1. ensures gps serial data is not kept waiting long  
  2. time the update parts of the loop and determine system update rate  
  3. determine how much of an impact sensor updates are (percentage of time reading sensors vs updating system)  
6. system update rate will be faster than sensor update rate  
  1. different than last quadcopter  
  2. will make the system model and kalman filter very important to get right  
7. sensor update rates will be kept reasonable, not super fast  
8. serial transmission offboard will be kept minimal  
  1. not every loop, but at predetermined rate (like 2Hz or something)  

## TODO  
- sd card recording - time how long reads and writes take  
- set up bluetooth  
- time an empty loop to verify processor speed  
- write Kalman filter class  
