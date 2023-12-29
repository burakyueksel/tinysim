# tinysim
A tiny simulator for aerial robots.
It is written in c++.

## Functions

### Controls
### Fusion
### Geometry
### Parameters
### Physics
### Sensors

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
