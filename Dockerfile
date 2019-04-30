FROM duckietown/rpi-duckiebot-base:master18

RUN [ "cross-build-start" ]

RUN apt-get update

RUN pip install --upgrade pip

RUN apt-get install python-smbus
RUN apt-get install i2c-tools


COPY requirements.txt /requirements_light_sensor.txt

RUN python -m pip install -r /requirements_light_sensor.txt


#Copy the package
RUN /bin/bash -c "source /home/software/catkin_ws/devel/setup.bash; cd /home/software/catkin_ws/; catkin_make"
COPY light_sensor /home/software/catkin_ws/src/light_sensor
RUN /bin/bash -c "source /home/software/catkin_ws/devel/setup.bash; cd /home/software/catkin_ws/; catkin_make"

RUN [ "cross-build-end" ]

CMD /bin/bash -c "source /home/software/catkin_ws/devel/setup.bash; roslaunch light_sensor light_sensor.launch veh:=$HOSTNAME" ]









