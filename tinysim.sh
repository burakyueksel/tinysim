#!/bin/bash

# Check if Makefile exists and clean files
if [ -f Makefile ]; then
    make clean_files
fi

# Run cmake
cmake .

# Run make with 8 cores
make -j8

# Run the simulation
./build/tinysim

# Run the Python script. Log file should be in the same folder level as this .sh file
python3 analyze/plotLog.py