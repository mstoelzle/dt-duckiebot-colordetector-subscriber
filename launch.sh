#!/bin/bash

set -e

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------
roslaunch colordetector_subscriber colordetector_subscriber.launch vehicle_name:=$VEHICLE_NAME color:=$COLOR
