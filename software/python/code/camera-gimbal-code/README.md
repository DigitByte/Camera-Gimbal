# Camera Gimbal

## Setup CAN ID's 
Run the `MYACTUATOR Setup Software 250206` software, which can be found here: https://www.myactuator.com/downloads-hseries
- Open software
- Enter `1` for communication id 
- Click on `Set ID`
- Click on `connect`

Repeat these steps for node ID's 2 and 3. Motor ID's are numbered 1, 2, 3, ordered from base to tool end. 

## Tuning Gains 
Position loop 
P: 0.05
I: 0 
D: 10 

Velocity gains
P: 0.007
I: 0.0001 
D: 0 

## Setup CAN
Note that the `can1` will vary depending on setup. For me it was `can1` because the default `can0` was their onboard CAN port that I was not using. I used the ODrive can-usb device.
```
sudo ip link set can1 up type can bitrate 1000000
```

## Wiring Mapping
```
RDK Pins: 
()    ()    (gnd) ...
(vcc) (sda) (scl) ...

IMU Pins: 
vcc 
gnd 
scl 
sda 
```

## Roll Pitch Yaw Directions
```
Roll pitch yaw directions 

roll - node 3 
pitch - node 2 
yaw - node 1 

-------
|     |   ---> roll
-------

   ^       
   |     pitch 
-------
|     |   
-------

-------
|  x  |  yaw 
-------
```

## CAN and Power Wiring
All CAN and power wiring was done with parallel connection. Main red and black lines split to the 3 motors. The CAN yellow and white lines split to the 3 motors. CAN cables used 26 awg cables. The power can be 24V-48V. 