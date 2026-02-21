FROM espressif/idf:v5.5.1

ARG MICROPYTHON_TAG=v1.27.0

RUN apt-get update && apt-get install -y \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Clone MicroPython and initialize
RUN git clone --depth 1 --branch ${MICROPYTHON_TAG} \
    https://github.com/micropython/micropython.git /opt/micropython \
    && cd /opt/micropython \
    && git submodule update --init --depth 1 lib/berkeley-db-1.xx \
    && git submodule update --init --depth 1 lib/micropython-lib \
    && make -C mpy-cross

ENV MICROPYTHON_DIR=/opt/micropython

WORKDIR /workspace
