# Instructions on how to connect with Howard

to launch the p2os driver run 

```bash
rosrun p2os_driver p2os_driver _port:=/dev/ttyUSB0
```
but first check that `/dev/ttyUSB0` is a valid device, you may find only `/dev/ttypUSBx` is available



## Setup

1. `roscore`
2. `rqt_graph`
3. `rosrun p2os_driver p2os_driver _port:=/dev/ttyUSB0`
4. `rosrun urg_node urg_node`
	- check that the sensor is detected as /dev/ttyACM0
5. other stuff
