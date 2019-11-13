# RH3 - Color detector subscriber

## Build
`dts devel build -f --arch amd64`

## Find IPs
### Laptop IP
`ifconfig`

### Duckiebot IP
`ping maxicar.local`

## Run:
`docker run -it --rm  --net host -e VEHICLE_NAME=maxicar -e ROS_MASTER_URI=http://MY_ROBOT_IP:11311/ -e ROS_IP=MY_IP -v /Users/maximilianstoelzle/Documents/ethz/AMoD/data:/data duckietown/dt-duckiebot-colordetector-subscriber:v1-amd64`

## Utils

### Access ROS-enabled container
`docker run -it --rm --net host -e VEHICLE_NAME=maxicar -e ROS_MASTER_URI=http://MY_ROBOT_IP:11311/ -e ROS_IP=MY_IP duckietown/dt-ros-commons:daffy-amd64 /bin/bash`

`rostopic list`

### Show GUI
`dts start_gui_tools maxicar --base_image duckietown/dt-core:daffy-amd64`

`rqt_image_view`