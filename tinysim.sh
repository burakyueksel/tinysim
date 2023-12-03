#!/bin/bash

# Check if Makefile exists and clean files
if [ -f Makefile ]; then
    make clean_files
fi

# Run cmake
cmake .

# Run make
make

# Run the simulation
./build/tinysim

# Run the Python script
#python3 ./analyse/plotSim.py