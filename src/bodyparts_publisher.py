#!/usr/bin/env python3

# -- IMPORT --
import numpy as np
import cv2
import torch
# Ros
import rospy
import cv_bridge
import ros_numpy
from sensor_msgs.msg import Image, CompressedImage
# Ros bodyparts
from helper_refinenet.resnet import rf_lw50, rf_lw101, rf_lw152
from helper_refinenet.helpers import prepare_img


class Bodyparts:
    def __init__(self):
        # Parameter
        self.camera_topic = rospy.get_param('/bodyparts/camera/topic')
        self.interface_topic = rospy.get_param('/bodyparts/interface/topic')
        self.visualization_topic = rospy.get_param('/bodyparts/visualization/topic')
        self.visualization_activated = rospy.get_param('/bodyparts/visualization/activated')
        self.gpu = rospy.get_param('/bodyparts/gpu')
        self.model_type = rospy.get_param('/bodyparts/model')

        # Init
        # -- Bridge & cuda
        self.bridge = cv_bridge.CvBridge()
        torch.cuda.set_device(self.gpu)

        # -- Model
        if(self.model_type == 50):
            self.model = rf_lw50(7, pretrained=True).eval().cuda(self.gpu)
        elif(self.model_type == 101):
            self.model = rf_lw101(7, pretrained=True).eval().cuda(self.gpu)
        elif(self.model_type == 152):
            self.model = rf_lw152(7, pretrained=True).eval().cuda(self.gpu)
        else:
            raise KeyError('Wrong model type -> correct config file ("50", "101", or "152" only)')

        # Publisher
        # -- Mask
        self.pub_mask = rospy.Publisher(self.interface_topic, CompressedImage, queue_size=1)

        # -- Visualization
        if self.visualization_activated:
            self.pub_visualization = rospy.Publisher(self.visualization_topic, Image, queue_size=1)

        # Subscriber
        rospy.Subscriber(self.camera_topic, CompressedImage, self._callback, queue_size=1)
        
        # Feedback
        print("Body segmentation publisher up and running")


    # Callback function
    def _callback(self, msg):

        t_start = rospy.get_time()

        # Get image
        image = cv2.cvtColor(self.bridge.compressed_imgmsg_to_cv2(msg), cv2.COLOR_BGR2RGB)
        
        # Calculate mask
        with torch.no_grad():
            image_tensor = torch.tensor(prepare_img(image).transpose(2,0,1)[None]).float()
            image_tensor = image_tensor.cuda(self.gpu)

            mask = self.model(image_tensor)[0].data.cpu().numpy().transpose(1,2,0)
            mask = cv2.resize(mask, image.shape[:2][::-1], interpolation=cv2.INTER_CUBIC)
            mask = mask.argmax(axis=2).astype(np.uint8)
        
         # Visualize results
        if self.visualization_activated:
            image[:,:,0][mask == 0] = 0
            image[:,:,1][mask == 0] = 0
            image[:,:,2][mask == 0] = 0
            self.pub_visualization.publish(ros_numpy.msgify(Image, image, encoding='8UC3'))

        # Publish results
        print('Body detection successful. Current Hz-rate:\t' + str(1/(rospy.get_time() - t_start)))
        self.pub_mask.publish(self.bridge.cv2_to_compressed_imgmsg(mask, dst_format='png'))
       

if __name__ == '__main__':

    rospy.init_node('bodyparts_publisher')
    body = Bodyparts()
    rospy.spin()