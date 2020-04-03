# ROS node for real-time body parts detection #

This is a ROS implementation of a light-weight RefineNet neural network trained on the body parts data set. The NN is capable of detecting human body parts and can differentiate between heads, torsos, upper arms, lower arms, upper legs, and lower legs with a mean intersection-over-union (IoU) score of 0.649 ([Nek18](https://github.com/DrSleep/light-weight-refinenet)).

This node is part of a larger project with the objective to enable object-independent human-to-robot handovers using robotic vision. The code for this project can be found [here](https://github.com/patrosAT/human_robot_handover_ros).

The node can be implemented as publisher, service, or action. See below for more information.

* **Input:** RGB image: [sensor_msgs/CompressedImage](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/CompressedImage.html)
* **Output:** Mask (0 background, 1-6 body parts): [sensor_msgs/CompressedImage](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/CompressedImage.html)

#### Example from a frontal perspective (full body): ####
<img src="./imgs/bodyFront.png" width="500"/>

#### Example from a lateral perspective (arm and hand only): ####
<img src="./imgs/bodySide.png" width="500"/>


## Getting Started ##

The code has been tested with Python 3.6.


### Hardware Requirements ###

* RGB camera *(for this project a [realsense D435](https://www.intelrealsense.com/depth-camera-d435/) was used)*
* GPU >= 2 GB


### Software Requirements ###

**ATTENTION: This package requires the [ROS](https://www.ros.org/) operating system!**

* Python: see [requirements.txt](requirements.txt)
* ROS packages:
```
rospy
actionlib
sensor_msgs
cv_bridge
ros_numpy
```


### Launch ###

The ROS package contains 3 launch files: publisher, service an action. 

* **[Publisher](launch/bodyparts_publisher.launch):** Publishes a mask every time a new image is published by the camera.
* **[Serivce](launch/bodyparts_service.launch):** Returns a mask upon service call.
* **[Action](launch/bodyparts_action.launch):** Returns a mask upon client call.

The input/output is identical for a all three nodes:
* **Input:** RGB image: [sensor_msgs/CompressedImage](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/CompressedImage.html)
* **Output:** Mask (0 background, 1-6 body parts): [sensor_msgs/CompressedImage](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/CompressedImage.html)


## Configuration ##

The initial configuration can be changed by adapting the [bodyparts.yaml](cfg/bodyparts.yaml) file:

**Camera:** 
* **topic:** Rostopic the publisher is subscribing to. Altering this configuration has no impact on the service and action nodes.

**Interfaces:**
* **topic:** Rostopic the publisher node is publishing to.
* **service:** Rosservice for interacting with the service node.
* **action:** Rostopic for interacting with the action node.

**Visualization:** The visualization mode publishes a color-inverted copy (BGR) of the original RGB image with the background blacked out. Please be aware that turning on the visualization increases computing time and network utilization substantially.

* **topic:** Rostopic the node is publishing to (visualization).
* **activated:** Turn on/off visualization: *use only keywords **"True"** or **"False"***

**GPU:**
* **gpu:** ID of the GPU (*only usable if more than one GPU is available*).

**Model:**
* **model:** Number of NN-layers ([RefineNet](https://arxiv.org/abs/1611.06612)): *use keywords **"50"**, **"101"** or **"152"** only*.


## Acknowledgments ##

The ROS node is powered by the pytorch implementation of [DrSleep](https://github.com/DrSleep). For more information on RefineNet please refer to the following [paper](https://arxiv.org/abs/1611.06612) or the [github repository](https://github.com/DrSleep/light-weight-refinenet).


## License ##

This project is licensed under the 4-clause BSD License.