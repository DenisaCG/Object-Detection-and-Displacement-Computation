# Object Detection and Displacement Computation

### Project Description
A duckietown project involving the object detection of duckies (yellow, rubber toy) and other duckiebots (moving robots), as well as computing the displacement to each detected object, all while manually driving a duckiebot through the designed duckietown.

### Pre-requisites
1. Creating a ROS workspace
```
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make
$ source devel/setup.bash
```
2. Enter your ROS workspace (that's where you will make all the changes, add code, etc)
```
$ cd ~/catkin_ws/src
```
3. Copy our Object Detection and Displacement Computation ROS package
```
git clone https://github.com/DenisaCG/Object-Detection-and-Displacement-Computation.git
```
4. Configure ROS package in order to enable running it 
```
$ cd ~/catkin_ws
$ catkin_make
$ source devel/setup.bash (Must be run on every shell you are using for ROS from now on.)
```

## Running the algorithm of object detection and displacement computation:
- In shell 1: 
```
$ rosrun image_transport republish compressed in:=/[duckiebot_name]/camera_node/image raw out:=/[duckiebot_name]/camera_node/image/raw
```
- In shell 2:
```
$ roscd Object-Detection-and-Displacement-Computation/script/
$ python detect_and_publish.py
```
- In shell 3:
```
$ dts start_gui_tools [duckiebotname]
# rqt_image_view
```

## Tools used for project creation
- [Dataset of images](https://github.com/duckietown/duckietown-objdet/tree/master/duckie_data/training-images) (containing 1000 images took around duckietown)
- [VoTT](https://github.com/microsoft/VoTT) (annotation tool)
- [Roboflow](https://roboflow.com/) (dataset creation)
- [Yolov5](https://blog.roboflow.com/how-to-train-yolov5-on-a-custom-dataset/) (object detection model) 
- [ROS Noetic](http://wiki.ros.org/noetic)

## Contributors 
- Denisa Checiu
- Kuranage Roche Rayan Ranasinghe
- Assylbek Sakenov

--- 
Object Detection Algorithm base on https://github.com/ldw200012/duckietown_yolov5. 
