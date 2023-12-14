# tinysim
A tiny simulator for aerial robots.
It is written in c++.

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

![Alt text](image.png)