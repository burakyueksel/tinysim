# tinysim
A tiny simulator for aerial robots.
It is written in c++.

## Function Declerations
Function declerations are done in respective header files, and all are collected under include/.

## Functions

### Controls
Here I collect different control algorithms used in aerial robotics. See src/controls for the implementation.
- [x] A PID controller for altitude control with its reference dynamics.
- [x] An INDI controller for attitude rate control.
- [x] A quaternion based tilt prioritizing attitude control.
- [x] A PID conroller for horizontal position control with its reference dynamics.
- [ ] A full geometric control in SE3.

### Flatness
- [ ] Differential flatness functions for 1-1 mapping between states, trajectories and the inputs.

### Fusion
Here I collect sensor fusion algorithms. See src/fusion for the implementation.

### Geometry
Especially the SO3 definitions. See src/geometry for the implementation.

### Parameters
All parameters are defined here. See src/parameters for the implementation.

### Physics
The rules of the physics that governs the motion of the robot are implemented here. See src/physics for the implementation.

### Sensors
Motion is sensed through sensors before used in control. Sensor models are defined here. See src/sensors for the immplementation.

- [x] IMU
- [x] Barometer
- [ ] GNSS receiver
- [ ] Onboard camera
- [ ] Radar
- [ ] Lidar

### Filters
Various filtering methods are needed and hence implemented. See src/filters for the implementation.

## Building
### Run everything in one go

```console
./tinysim.sh
```

### Compile

To run the code:

```console
cmake .
make
./build/tinysim
```


### Clean Binaries

```console
make clean_files
```

### Plotting

This simulator generates a log.txt file. Under /analyze you will find a python script for reading and plotting the data in it. For example:

![image](https://github.com/burakyueksel/tinysim/assets/40430575/69bd08bb-b50a-42de-bdf3-ce49ef340380)
