# Visual SLAM on Raspberry Pi 5 with ORB SLAM3, ROS2 Humble, and RViz2

This project aims to achieve Visual SLAM using ORB SLAM3, ROS2 Humble, and RViz2 on Raspberry Pi 5 with Bookworm OS and Raspberry Camera Module 3, similar to the output in the video. Below are the detailed steps for setting up the environment and software on Raspberry Pi.

[![Visual SLAM Demo](https://img.youtube.com/vi/uCHXowZHt8c/0.jpg)](https://www.youtube.com/watch?v=uCHXowZHt8c)

## Table of Contents
- [Swap Configuration](#swap-configuration)
- [Installing ROS2 Humble](#installing-ros2-humble)
- [GCC Version Downgrade](#gcc-version-downgrade)
- [Installing Required Libraries](#installing-required-libraries)
- [Installing Image Transport and cv_bridge](#installing-image-transport-and-cv_bridge)
- [Installing Pangolin](#installing-pangolin)
- [Installing ORB SLAM3](#installing-orb-slam3)
- [Publishing ORB SLAM3 Data in ROS2](#publishing-orb-slam3-data-in-ros2)
- [Sourcing the Installations](#sourcing-the-installations)
- [Image Publisher Setup](#image-publisher-setup)
- [X11 Session Setup for RViz2](#x11-session-setup-for-rviz2)
- [Running the Setup](#running-the-setup)

## Swap Configuration
First, increase the swap space to make installations smoother:

```bash
# Create 8 GB swap space (you can allocate more if needed):
sudo fallocate -l 8G /swapfile

# Set swap file permissions:
sudo chmod 600 /swapfile

# Enable the swap file:
sudo mkswap /swapfile
sudo swapon /swapfile

# Verify swap:
swapon --show

# Make it permanent by adding to /etc/fstab:
echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab
```

## Installing ROS2 Humble
Since ROS2 Humble cannot be directly installed on Debian 12, use the following repository (approx. 3-hour process):

1. Clone the repository to your Raspberry Pi 5:
    ```bash
    git clone https://github.com/ozandmrz/ros2_raspberry_pi_5
    cd ros2_raspberry_pi_5
    ```

2. Make the installation script executable:
    ```bash
    chmod +x ros2_humble_install.sh
    ```

3. Run the script:
    ```bash
    ./ros2_humble_install.sh
    ```
    This script will:
    - Install necessary dependencies.
    - Set up a new ROS2 workspace.
    - Download the ROS2 Humble source code.
    - Compile ROS2.
    - Configure environment variables to source the setup file automatically.

4. Source the environment:
    ```bash
    echo "source ~/ros2_humble/install/local_setup.bash" >> ~/.bashrc
    source ~/.bashrc
    ```

5. Install additional Ament dependencies:
    ```bash
    sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-keys F42ED6FBAB17C654
    echo "deb http://repo.ros2.org/ubuntu/main bookworm main" | sudo tee /etc/apt/sources.list.d/ros2.list
    sudo apt update
    sudo apt install python3-pip python3-setuptools python3-colour python3-rosdep python3-ament-package
    ```

### Troubleshooting:
If any issues arise:
```bash
sudo dpkg --remove --force-all python3-catkin-pkg
sudo dpkg --remove --force-remove-reinstreq python3-rospkg python3-rosdistro
sudo apt --fix-broken install
sudo apt install python3-catkin-pkg-modules python3-rospkg-modules python3-rosdistro-modules
sudo apt install libopencv-dev python3-opencv
```

## GCC Version Downgrade
To install ORB SLAM3 and Sophus correctly, downgrade GCC from version 12 to 11:
```bash
sudo apt-get update
sudo apt-get install gcc-11 g++-11
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-11 60 --slave /usr/bin/g++ g++ /usr/bin/g++-11
sudo update-alternatives --config gcc
# Choose auto mode (0) to set /usr/bin/gcc-11
```

Verify the version change:
```bash
gcc --version
```

## Installing Required Libraries
```bash
sudo apt-get update
sudo apt-get install build-essential cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
sudo apt-get install libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libdc1394-22-dev libxvidcore-dev libx264-dev
sudo apt-get install libatlas-base-dev gfortran python3-dev libeigen3-dev libboost-all-dev libsuitesparse-dev
sudo apt-get install libopencv-dev libglew-dev cmake libpython2.7-dev python2.7 libboost-python-dev
```

## Installing Image Transport and cv_bridge
```bash
mkdir -p ~/my_ros2_workspace/src
cd ~/my_ros2_workspace/src
git clone https://github.com/ros-perception/image_common.git
cd image_common
git checkout humble
cd ~/my_ros2_workspace/src
git clone https://github.com/ros-perception/vision_opencv.git
cd vision_opencv/cv_bridge
git checkout humble
cd ~/my_ros2_workspace
colcon build --symlink-install
```

## Installing Pangolin
```bash
cd ~
git clone https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin
mkdir build
cd build
cmake ..
make -j4
sudo make install
```

## Installing ORB SLAM3
```bash
git clone https://github.com/ozandmrz/ORB_SLAM3
cd ~/ORB_SLAM3

# Build Thirdparty/DBoW2:
cd Thirdparty/DBoW2
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j2

# Build Thirdparty/g2o:
cd ../../g2o
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j2

# Build Thirdparty/Sophus:
cd ../../Sophus
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j2
sudo make install

# Uncompress the vocabulary:
cd ../../../Vocabulary
tar -xf ORBvoc.txt.tar.gz

# Build ORB_SLAM3:
cd ../build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4
```

## Publishing ORB SLAM3 Data in ROS2
```bash
mkdir -p ~/ros2_pose/src
cd ~/ros2_pose/src
git clone https://github.com/ozandmrz/orb_slam3_ros2_mono_publisher.git
cd ~/ros2_pose
rosdep install --from-paths src --ignore-src -r -y
colcon build
echo "source ~/ros2_pose/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Sourcing the Installations
```bash
echo 'export CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH:/opt/ros/humble' >> ~/.bashrc
echo 'export CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH:~/ORB_SLAM3:~/ORB_SLAM3/Thirdparty/Sophus' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/ORB_SLAM3/lib' >> ~/.bashrc
echo "source ~/ros2_humble/install/local_setup.bash" >> ~/.bashrc
echo 'export CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH:~/ORB_SLAM3/Thirdparty/DBoW2:~/ORB_SLAM3/Thirdparty/g2o' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/ORB_SLAM3/Thirdparty/DBoW2/lib:~/ORB_SLAM3/Thirdparty/g2o/lib' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib' >> ~/.bashrc
source ~/.bashrc
```

## Image Publisher Setup
```bash
mkdir -p ~/image_publisher/src
cd ~/image_publisher/src
git clone https://github.com/ozandmrz/ros2_image_publisher.git
cd ~/image_publisher
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
echo "source ~/image_publisher/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## X11 Session Setup for RViz2
Install X11 apps to display RViz2 properly on Raspberry Pi:
```bash
sudo apt update
sudo apt install x11-apps x11-xserver-utils xserver-xorg-video-fbdev
```

## Running the Setup
In separate terminals, execute the following commands:

1. Run ORB SLAM3 and publish topics:
    ```bash
    ros2 run orbslam3_pose mono /home/rasp7/ORB_SLAM3/Vocabulary

/ORBvoc.txt /home/rasp7/ORB_SLAM3/Examples/RGB-D/TUM1.yaml /dev/video0
    ```

2. Run the camera:
    ```bash
    ros2 launch image_publisher image_publisher.launch.py
    ```

3. Launch RViz2 for visualization:
    ```bash
    export XDG_SESSION_TYPE=x11 
    ros2 launch orbslam3_pose rviz.launch.py
    ```

