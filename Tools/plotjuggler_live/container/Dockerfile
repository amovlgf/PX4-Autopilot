ARG AGENT_BUILD_IMAGE=ubuntu:22.04
ARG ROS_BASE_IMAGE=ubuntu:22.04

FROM ${AGENT_BUILD_IMAGE} AS agent-builder
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        build-essential ca-certificates cmake git libasio-dev libssl-dev \
        libtinyxml2-dev python3 \
    && rm -rf /var/lib/apt/lists/*
COPY build-agent.sh /tmp/build-agent.sh
RUN bash /tmp/build-agent.sh

FROM ${ROS_BASE_IMAGE}
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
LABEL org.opencontainers.image.title="PX4 PlotJuggler live monitor" \
      org.opencontainers.image.description="ROS Humble, PlotJuggler ROS plugin and Micro XRCE-DDS Agent 2.4.2"
ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=humble \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    LIBGL_ALWAYS_SOFTWARE=1 \
    QT_X11_NO_MITSHM=1 \
    QT_QPA_PLATFORM=xcb
# Official ros/rosdistro ros.key, fingerprint C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654.
# SHA256: 4a91c49af0d6f0016108b93698782b596c27ccd836937e18e0e36c3347dc602f.
COPY ros-archive-keyring.gpg /usr/share/keyrings/ros-archive-keyring.gpg
RUN printf 'deb [arch=%s signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main\n' "$(dpkg --print-architecture)" \
        > /etc/apt/sources.list.d/ros2.list \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
        bash build-essential ca-certificates cmake coreutils git iproute2 \
        libgl1-mesa-dri libgl1-mesa-glx libssl3 libtinyxml2-9 libxcb-xinerama0 \
        procps python3-colcon-common-extensions python3-numpy python3-pytest \
        python3-yaml ros-humble-ament-cmake ros-humble-geometry-msgs \
        ros-humble-ros-base ros-humble-plotjuggler-ros ros-humble-rclpy ros-humble-rmw-fastrtps-cpp \
        ros-humble-rosidl-default-generators ros-humble-rosidl-default-runtime \
        ros-humble-std-msgs util-linux xauth \
    && mkdir -p /usr/local/share/px4-plotjuggler-live \
    && dpkg-query -W -f='${Package}\t${Version}\n' \
        > /usr/local/share/px4-plotjuggler-live/packages.tsv \
    && rm -rf /var/lib/apt/lists/*
COPY --from=agent-builder /opt/micro-xrce-dds-agent /opt/micro-xrce-dds-agent
COPY MicroXRCEAgent /usr/local/bin/MicroXRCEAgent
# Agent v2.4.2 returns 1 when printing usage; require its real usage output.
RUN chmod 0755 /usr/local/bin/MicroXRCEAgent \
    && { /usr/local/bin/MicroXRCEAgent --help > /tmp/agent-help.txt 2>&1; test "$?" -eq 1; } \
    && grep -q "Available arguments" /tmp/agent-help.txt \
    && rm /tmp/agent-help.txt \
    && source /opt/ros/humble/setup.bash \
    && ros2 pkg prefix plotjuggler \
    && ros2 pkg prefix plotjuggler_ros \
    && python3 -c 'import rclpy, yaml, numpy'
# Runtime scripts source ROS and the generated workspace, with the invoking UID.
# Project sources and generated px4_msgs are supplied by bind mounts.
WORKDIR /work
ENTRYPOINT []
CMD ["bash"]
