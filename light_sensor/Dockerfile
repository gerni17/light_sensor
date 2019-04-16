FROM duckietown/rpi-ros-kinetic-roscore:master18

RUN [ "cross-build-start" ]

RUN apt-get update

RUN pip install --upgrade pip

RUN apt-get install python-smbus
RUN apt-get install i2c-tools
#RUN i2cdetect -y 1

COPY requirements.txt .

RUN python -m pip install -r requirements.txt

#Copy python file

COPY light_sensor.py .

ENTRYPOINT [ "python" ]

RUN [ "cross-build-end" ]

CMD [ "light_sensor.py" ]









