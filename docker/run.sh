sudo xhost +local:*

sudo docker run -it --rm --network host -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix/:/tmp/.X11-unix/ \
    -v /tmp/argus_socket:/tmp/argus_socket \
    solo-image
