#!/bin/bash
xhost +local:docker
docker run -it --rm \
    --network host \
    -e DISPLAY=$DISPLAY \
    -e XDG_RUNTIME_DIR=/tmp/runtime-root \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    --device /dev/dri \
    -e "CYCLONEDDS_URI=<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"enp1s0f0\"/></Interfaces></General></Domain></CycloneDDS>" \
    -v $(pwd):/workspace:z \
    acre_slam