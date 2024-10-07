# cd ./docker-configuration
# sudo docker compose up -d ros-px4-humble
# sudo docker compose down ros-px4-humble
# cd ..

sudo docker run  \
  --device /dev/dri:/dev/dri \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  --cpus="4" \
  --memory="8g" \
  --name ros-px4-humble \
  ros-px4-humble 
  


# sudo apt-get install -y \
#     vainfo \
#     libva-dev \
#     libva-intel-driver \
#     libva-utils
#     beignet-opencl-icd \
#     clinfo