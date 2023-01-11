# urs_aerial
The repository contains exercises and templates for the seminar on the course Robotics systems control - aerial robotics.

## Tello driver installation

For ROS Noetic there are few prerequisites:

```sh
sudo apt install libx264-dev python-yaml
sudo pip install av

cd catkin_ws/src
git clone https://github.com/tilk/h264_image_transport.git
git clone https://github.com/anqixu/tello_driver.git
git clone https://github.com/anqixu/TelloPy.git

cd TelloPy
sudo -H pip install -e .
cd ..

cd h264_image_transport
rosdep install h264_image_transport
cd ..

catkin build -c
```

Build the workspace. Run the driver with:

```sh
roslaunch tello_driver tello_node.launch
```