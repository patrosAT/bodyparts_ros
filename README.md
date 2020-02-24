# ROS node for real-time bodyparts detection #

This is a ROS implementation of the RefineNet Neural Network trained on the bodyparts dataset.

#### Input ####

**RGB compressed image:** [sensor_msgs/CompressedImage](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/CompressedImage.html)

#### Output ####

**Compressed mask:** [sensor_msgs/CompressedImage](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/CompressedImage.html) (0 indicates background, 1 to 6 the individual bodyparts)

## Getting Started ##

### Dependencies ###

The models have been tested with Python 2.7 and 3.6.

#### Hardware ####

* RGBD Camera
* *Recommended:* GPU >= 2000MiB
 
#### Python3 / pip3 ####
```
numpy
cv2
torch
```
#### Ros ####
```
rospy
actionlib
sensor_msgs
cv_bridge
ros_numpy
```

### Bilding ###

*Optional:* To maximize performance, use the 'release' build mode:
```
catkin_make -DCMAKE_BUILD_TYPE=Release
```

### Configuration ###

The initial setup can be changed by adapting the [bodyparts.yaml](cfg/bodyparts.yaml) file:

#### Camera ####
* **topic:** Rostopic the publisher node is subcribing to.

#### Interfaces ####
* **topic:** Rostopic the publisher node is publishing to *(please do not change)*.
* **service:** Rosservice for interacting with the service node *(please do not change)*.
* **action:** Rostopic for interacting with the action node *(please do not change)*.

#### Visualization ####

The visualization mode published the original image with the background blacked out. Please be aware that turing on the visualization increases computing time and network utilization substantially.

* **topic:** Topic the node is publishing to.
* **activated:** Turn on/off visualization: *use keywords "on" or "off"*.

#### GPU ###
* **gpu:** Number of the gpu.

#### Model ###
* **model:** Number of NN-layers: *use keywords "50", "101" or "152"*

### Launch ###

The ros package contains 3 launch files:
* **Publisher:** The [publisher](launch/bodyparts_publisher.launch) launch file starts a ros node that published a new mask every time a new rgb image is published.
* **Serivce:** The [serivce](launch/bodyparts_service.launch) launch file starts a ros service. 
* **Action:** The [action](launch/bodyparts_action.launch) launch file starts a ros action server.

## Acknowledgments

The ROS node is powered by the pytorch implementation of [DrSleep](https://github.com/DrSleep). For more information on RefineNet please refer to the original [paper](https://arxiv.org/abs/1611.06612) or the following [github repository](https://github.com/DrSleep/light-weight-refinenet)

## License

* **Academic:** This project is licensed under the 3-clause BSD License.
* **Commercial:** This project is licensed under the 3-clause BSD License. The underlying resouced might use different licence models. Contanct the respective authors for more information.
