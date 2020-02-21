# ROS node for real-time bodyparts detection

This is a ROS implementation of the RefineNet Neural Network trained on the bodyparts dataset.

**Input:** RGB image: [sensor_msgs/CompressedImage](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/CompressedImage.html)
**Output:** Mask: [sensor_msgs/CompressedImage](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/CompressedImage.html) indicating the bodyparts (0 for background, 1 to 6 for the individual bodyparts)

## Getting Started

### Dependencies

The models have been tested with Python 3.6.
 
#### Python3 / pip3
```
numpy
cv2
torch
```
#### Ros
```
rospy
actionlib
sensor_msgs
cv_bridge
```

### Configuration

The initial setup can be changed by adapting the [bodyparts.yaml](cfg/bodyparts.yaml) file:
* **rgb:** Camera topic the publisher node is subcribing to.
* **topic:** Topic the publisher node is publishing to.
* **service:** Topic the service node is subcribing & publishing to.
* **action:** Topic the action node is subcribing & publishing to.
* **model:** Number of NN-layers. Possible options are '50', '101', and '152'.

### Launch

The ros package contains 3 launch files:
* **Publisher:** The [publisher](launch/bodyparts_publisher.launch) launch file starts a ros node that published a new mask every time a new rgb image is published.
* **Serivce:** The [serivce](launch/bodyparts_service.launch) launch file starts a ros service. 
* **Action:** The [action](launch/bodyparts_action.launch) launch file starts a ros action server.

## RefineNet

The ROS node is powered by the pytorch implementation of [DrSleep](https://github.com/DrSleep). For more information on RefineNet please refer to the original [paper](https://arxiv.org/abs/1611.06612) or the following github [repository](https://github.com/DrSleep/light-weight-refinenet)

## License

**Academic:** The project is licensed under the 3-clause BSD License
**Commercial:** Please contact the author


