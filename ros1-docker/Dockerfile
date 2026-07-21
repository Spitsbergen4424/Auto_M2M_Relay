FROM ros:noetic-ros-base-focal

ARG DEBIAN_FRONTEND=noninteractive
ARG ENDPOINT_REF=993d366b8900bf9f3d2da444fde64c0379b4dc7c

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        git \
        netcat-openbsd \
        python3-catkin-tools \
    && ln -s /usr/bin/python3 /usr/local/bin/python \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /catkin_ws/src/ROS-TCP-Endpoint
RUN git init \
    && git remote add origin https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git \
    && git fetch --depth 1 origin "${ENDPOINT_REF}" \
    && git checkout --detach FETCH_HEAD

WORKDIR /catkin_ws
RUN /bin/bash -lc "source /opt/ros/noetic/setup.bash && catkin_make"

COPY endpoint_entrypoint.sh /endpoint_entrypoint.sh
RUN chmod +x /endpoint_entrypoint.sh

EXPOSE 10000
HEALTHCHECK --interval=5s --timeout=2s --start-period=10s --retries=12 \
    CMD nc -z 127.0.0.1 10000 || exit 1

ENTRYPOINT ["/endpoint_entrypoint.sh"]
