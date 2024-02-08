#!/bin/bash

for file in *.bag; do
    # Check if the file is a regular file
    if [ -f "$file" ]; then
        # Call the function for each .bag file
        name="${file%.*}"
        rosbags-convert "$file" --include-topic /ublox/fix
        ros2bag-convert "${name}/${name}.db3"
    fi
done