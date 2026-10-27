# Ubuntu 24.04 / ROS 2 Jazzy, multi-architecture manifest pinned on 2026-09-08.
ARG ROS_IMAGE=ros:jazzy-ros-base-noble@sha256:2589a8fba5257307857890173c069852c2abf913a0be7970f172478baecb09e4
FROM ${ROS_IMAGE} AS dev

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ARG DEBIAN_FRONTEND=noninteractive
ARG VIO_UID=1000
ARG VIO_GID=1000
ARG INSTALL_REALSENSE=false

# System Harmonic libraries are also needed by the user's existing PX4 binary.
# ros_gz remains the ROS/Gazebo bridge; no PX4, Agent or QGC is installed here.
RUN apt-get update && apt-get install -y --no-install-recommends \
      ca-certificates curl gnupg \
    && curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
      -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable noble main" \
      > /etc/apt/sources.list.d/gazebo-stable.list \
    && apt-get update && apt-get install -y --no-install-recommends \
      build-essential cmake git pkg-config \
      python3-colcon-common-extensions python3-rosdep python3-vcstool \
      python3-numpy python3-scipy python3-yaml python3-pytest \
      procps iproute2 util-linux usbutils \
      libgl1-mesa-dri libegl1 mesa-utils xvfb xauth \
      gz-harmonic ros-jazzy-ros-gz ros-jazzy-rviz2 \
    && case "${INSTALL_REALSENSE}" in \
      true) apt-get install -y --no-install-recommends \
        ros-jazzy-realsense2-camera ros-jazzy-realsense2-description ;; \
      false) ;; \
      *) echo 'INSTALL_REALSENSE must be true or false' >&2; exit 2 ;; \
    esac \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws
COPY src/ /ws/src/
# PX4 and VIO_ver1 are explicit external workspace dependencies.
# rosdep installs system dependencies; it does not provision these overlays.
RUN source /opt/ros/jazzy/setup.bash \
    && (test -f /etc/ros/rosdep/sources.list.d/20-default.list || rosdep init) \
    && rosdep update --rosdistro jazzy \
    && apt-get update \
    && rosdep install --from-paths src --ignore-src --rosdistro jazzy \
      --skip-keys "vio_msckf px4_msgs px4_ros2_cpp" -y \
    && mkdir -p /opt/vio \
    && dpkg-query -W > /opt/vio/apt-packages.tsv \
    && rm -rf /var/lib/apt/lists/*

# Ubuntu images may already contain a user with UID 1000. Reuse its numeric
# identity with a stable name inside this image; the host is never modified.
RUN if getent passwd "${VIO_UID}" >/dev/null; then \
      test "${VIO_UID}" != 0 \
      && userdel "$(getent passwd "${VIO_UID}" | cut -d: -f1)"; \
    fi \
    && (getent group "${VIO_GID}" >/dev/null || groupadd --gid "${VIO_GID}" vio) \
    && useradd --uid "${VIO_UID}" --gid "${VIO_GID}" --create-home --shell /bin/bash vio \
    && mkdir -p /ws/build /ws/install /ws/log \
    && chown -R "${VIO_UID}:${VIO_GID}" /ws

COPY scripts/ /ws/scripts/
COPY docker/entrypoint.sh /usr/local/bin/vio-entrypoint
COPY docker/build.sh /usr/local/bin/vio-build
COPY docker/smoke.sh /usr/local/bin/vio-smoke
RUN chmod 755 /usr/local/bin/vio-entrypoint /usr/local/bin/vio-build /usr/local/bin/vio-smoke \
    && echo 'source /usr/local/bin/vio-entrypoint' >> /home/vio/.bashrc

ENV ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1 \
    GZ_PARTITION=uav_vio GZ_IP=127.0.0.1 \
    CMAKE_BUILD_PARALLEL_LEVEL=2 MAKEFLAGS="-j2 -l2"
USER vio
ENTRYPOINT ["/usr/local/bin/vio-entrypoint"]
CMD ["bash"]

# Optional build snapshot with complete prebuilt workspaces supplied as named
# contexts. Keep their original absolute paths for symlink-install/RPATH.
FROM dev AS runtime-built
USER root
ARG PX4_WS
ARG VIO_ALGORITHM_WS
RUN test -n "${PX4_WS}" && test -n "${VIO_ALGORITHM_WS}"
COPY --from=px4_workspace --chown=vio:vio / ${PX4_WS}/
COPY --from=vio_estimator_workspace --chown=vio:vio / ${VIO_ALGORITHM_WS}/
COPY --from=px4_source --chown=vio:vio / /opt/px4-source/
ENV PX4_WS=${PX4_WS} VIO_ALGORITHM_WS=${VIO_ALGORITHM_WS} PX4_DIR=/opt/px4-source
USER vio
# Build application packages with actual PX4 source/hash checks. The optional
# pinned interface library is provided by the mounted dependency workspace.
RUN vio-build
