#!/bin/bash

# Check if build folder exists and clean files
folder_to_delete="build"
if [ -d "$folder_to_delete" ]; then
    rm -r "$folder_to_delete"
    echo "Old $folder_to_delete deleted. A new one will be created."
else
    echo "No $folder_to_delete folder exist, it will be created."
fi

# Create a build folder
mkdir build

# Go into the build folder
cd build

# Run cmake
cmake ..
echo "cmake is done."

# Run make with 8 cores
make -j8
echo "make is done."

# Come back to parent folder
cd ..

# Run the simulation
echo "simulating..."
./build/tinysim
echo "simulation finished."

# Run the Python script. Log file should be located in the same folder level as this .sh file
python3 analyze/plotLog.py
echo "analyzing..."