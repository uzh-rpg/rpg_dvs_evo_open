# #!/bin/bash

echo "First, we install the required apt packages"
sudo add-apt-repository ppa:inivation-ppa/inivation
sudo apt-get update
sudo apt-get install            \
    ros-$1-sophus               \
    ros-$1-pcl-ros              \
    ros-$1-eigen-conversions    \
    ros-$1-camera-info-manager  \
    ros-$1-image-view           \
    libcaer-dev                 \
    libfftw3-dev libfftw3-doc   \
    libglew-dev                 \
    libopencv-dev               \
    libyaml-cpp-dev             \
    python-catkin-tools         \
    ros-$1-camera-info-manager  \
    ros-$1-image-geometry



echo "Second, we clone the evo dependencies"
vcs-import < rpg_dvs_evo_open/dependencies.yaml
