FROM osrf/ros:noetic-desktop-full

COPY . /

WORKDIR /

RUN ./config.sh

CMD [ "bash" ]