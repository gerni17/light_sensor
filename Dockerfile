FROM duckietown/rpi-ros-kinetic-roscore:master18

RUN [ "cross-build-start" ]

RUN apt-get update

RUN pip install --upgrade pip

RUN apt-get install python-smbus
RUN apt-get install i2c-tools
#RUN i2cdetect -y 1

COPY requirements.txt .

RUN python -m pip install -r requirements.txt

#Copy the package

RUN mkdir -p /catkin-ws/src
RUN /bin/bash -c "source /opt/ros/kinetic/setup.bash; cd /catkin-ws/; catkin_make"
COPY light_sensor /catkin-ws/src/light_sensor
RUN /bin/bash -c "source /catkin-ws/devel/setup.bash; cd /catkin-ws/; catkin_make"

RUN [ "cross-build-end" ]

CMD /bin/bash -c "source /catkin-ws/devel/setup.bash; roslaunch light_sensor light_sensor.launch veh:=$HOSTNAME" ]









