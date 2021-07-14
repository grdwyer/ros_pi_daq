# ROS Pi DAQ

Package to use the gpios of the raspberry pi through ros

To run this create a config file with the desired pins using the [gpio number](https://pinout.xyz/) using `config/two_of_each.yaml` as a reference.
Replace the reference to the config file in `launch/daq_launch.launch.py'
Then launch
`ros2 launch daq_server daq_server.launch.py`

Alternatively configure pins using the service:
`ros2 service call /daq_server/configure_pins daq_interfaces/srv/ModifyGPIOSetup "{pin_numbers: [], operations: []}"`  
Where pin_numbers and operations are both yaml arrays, possible operations are currently:  
```
Set as Output - 1
Set as Input - 2
Remove configured pin - 3
```

If it is setup you can run this on another system and use the raspberry pis (remote gpio)[https://gpiozero.readthedocs.io/en/stable/remote_gpio.html]
capability. Just set the environment variable `export PIGPIO_ADDR='<raspberry pi ip address>'` before launching.

## Docker
### Build
```bash
docker build --pull --rm -f ./.docker/Dockerfile  -t gdwyer/ros_pi_daq:latest .
```

### Run for use  
Default command is now the daq server replace with bash if you don't want the server to run 
```bash
docker run -d \
    --net=host
    gdwyer/ros_pi_daq:latest
```

### Run for dev
```bash
docker run -it \
    --user=$(id -u $USER):$(id -g $USER) \
    --group-add sudo \
    --group-add gpio \
    --env="DISPLAY" \
    --env=QT_X11_NO_MITSHM=1 \
    --workdir="/dev_ws/src" \
    --volume="/home/$USER:/home/$USER" \
    --volume="/etc/group:/etc/group:ro" \
    --volume="/etc/passwd:/etc/passwd:ro" \
    --volume="/etc/shadow:/etc/shadow:ro" \
    --volume="/etc/sudoers.d:/etc/sudoers.d:ro" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    gdwyer/ros_pi_daq:latest bash
```
Full docker info [here](https://github.com/grdwyer/Robot-Assisted-Manufacturing/wiki/Docker)
