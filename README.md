# sensor
This is the repository for a sensor tcs 34725


### Running publisher
```bash
docker -H hostname.local run -it --rm --net host --privileged --name light-sensor surirohit/light-sensor
```

### Running subscriber
```bash
docker -H hostname.local exec -it light-sensor /bin/bash
```
Once inside container,
```bash
source /catkin_ws/devel/setup.bash
rostopic echo /hostname/light_sensor_node/sensor_data
```