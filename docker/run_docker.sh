image_name=hatpehda_noetic

echo "Starting docker hatpehda_noetic container..."
echo $PWD


# add --gpus all \ to use GPU

docker run --privileged --network host \
           --env="DISPLAY=$DISPLAY" \
           --env="QT_X11_NO_MITSHM=1" \
           --volume="$HOME/.Xauthority:$HOME/.Xauthority:rw" \
           --env XAUTHORITY=$HOME/.Xauthority \
           --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
           --volume="$PWD/../:$PWD/../" \
           -v /dev/dri:/dev/dri \
           -v /dev/bus/usb:/dev/bus/usb \
           -v /dev/input:/dev/input \
           --rm -it ${image_name}