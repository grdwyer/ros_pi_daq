# ROS Pi DAQ

Package to use the gpios of the raspberry pi through ros

To run this create a config file with the desired pins using the [gpio number](https://pinout.xyz/) using `config/two_of_each.yaml` as a reference.
Replace the reference to the config file in `launch/daq_launch.launch.py'
Then launch
`ros2 launch daq_server daq_server.launch.py`

If it is setup you can run this on another system and use the raspberry pis (remote gpio)[https://gpiozero.readthedocs.io/en/stable/remote_gpio.html]
capability. Just set the environment variable `export PIGPIO_ADDR='<raspberry pi ip address>'` before launching.

## Docker
### Build
```bash
docker build --pull --rm -f ./.docker/Dockerfile  -t gdwyer/ros_pi_daq:latest .
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
    gdwyer/ros_pi_daq:latest
```

### Run for use
```bash
docker run -it \
    --workdir="/dev_ws" \
    --volume="/home/$USER:/home/$USER" \
    --volume="/etc/group:/etc/group:ro" \
    --volume="/etc/passwd:/etc/passwd:ro" \
    --volume="/etc/shadow:/etc/shadow:ro" \
    --volume="/etc/sudoers.d:/etc/sudoers.d:ro" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    gdwyer/ros_pi_daq:latest
```
Full docker info [here](https://github.com/grdwyer/Robot-Assisted-Manufacturing/wiki/Docker)
