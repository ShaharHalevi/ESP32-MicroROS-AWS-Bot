FROM osrf/ros:jazzy-desktop

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash","-lc"]

# Basic dev tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    git build-essential python3-pip python3-colcon-common-extensions \
    python3-vcstool ros-dev-tools sudo udev xauth \
 && rm -rf /var/lib/apt/lists/*

# ---------- Non-root user setup (UID/GID collision-safe) ----------
ARG USERNAME=robot
ARG UID=1000
ARG GID=1000

# Decide effective user & group; reuse existing UID/GID if they already exist
# Also ensure /home/robot exists and is owned by that UID:GID so we can use a stable path.
RUN set -eux; \
    # Resolve/ensure group
    if getent group "${GID}" >/dev/null; then \
        GROUP_NAME="$(getent group ${GID} | cut -d: -f1)"; \
    else \
        GROUP_NAME="${USERNAME}"; \
        groupadd -g "${GID}" "${GROUP_NAME}"; \
    fi; \
    # Resolve/ensure user
    if getent passwd "${UID}" >/dev/null; then \
        EFFECTIVE_USER="$(getent passwd ${UID} | cut -d: -f1)"; \
        # Make sure that user has sudo/dialout/video
        usermod -aG sudo,dialout,video "${EFFECTIVE_USER}" || true; \
    else \
        EFFECTIVE_USER="${USERNAME}"; \
        useradd -m -u "${UID}" -g "${GID}" -s /bin/bash "${EFFECTIVE_USER}"; \
        usermod -aG sudo,dialout,video "${EFFECTIVE_USER}"; \
    fi; \
    echo "${EFFECTIVE_USER} ALL=(ALL) NOPASSWD:ALL" > "/etc/sudoers.d/${EFFECTIVE_USER}"; \
    # Stable home for your workspace: /home/robot (even if EFFECTIVE_USER != robot)
    mkdir -p /home/${USERNAME}/ws; \
    chown -R "${UID}:${GID}" /home/${USERNAME}; \
    # If EFFECTIVE_USER has its own home, keep both in sync (optional symlink)
    if [ "${EFFECTIVE_USER}" != "${USERNAME}" ]; then \
        HOME_EXISTING="$(getent passwd ${EFFECTIVE_USER} | cut -d: -f6)"; \
        # create a convenience symlink from /home/robot/home to existing home (optional)
        ln -sfn "${HOME_EXISTING}" "/home/${USERNAME}/home_${EFFECTIVE_USER}"; \
    fi; \
    # Persist for later Dockerfile steps
    echo "EFFECTIVE_USER=${EFFECTIVE_USER}" >> /etc/environment

# Set a friendly prompt and source ROS in shells for BOTH homes
RUN set -eux; \
    for H in "/home/${USERNAME}" "$(getent passwd $(. /etc/environment; echo ${EFFECTIVE_USER}) | cut -d: -f6)"; do \
        [ -d "$H" ] || continue; \
        echo 'export PS1="\\u@robot-pc:\\w\\$ "' >> "$H/.bashrc"; \
        echo 'source /opt/ros/jazzy/setup.bash' >> "$H/.bashrc"; \
        chown -R ${UID}:${GID} "$H"; \
    done

# Workspace lives here regardless of the underlying account name
WORKDIR /home/${USERNAME}/ws

# Entrypoint
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Switch to the effective user (resolved above)
# Note: expand EFFECTIVE_USER from /etc/environment at build time:
ARG EFFECTIVE_USER
RUN export $(cat /etc/environment | xargs) && echo "Using user: ${EFFECTIVE_USER}"
USER ${EFFECTIVE_USER}

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]

