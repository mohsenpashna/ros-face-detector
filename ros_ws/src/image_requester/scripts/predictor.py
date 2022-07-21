#!/usr/bin/env python3
# Software License Agreement (BSD License)

import numpy as np
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
import requests
import json
import cv2
from common import *


font = cv2.FONT_HERSHEY_SIMPLEX


def callback(data):
    try:
        res = requests.post(url='http://172.17.0.2/image',
                        data=data.data,
                        headers={'Content-Type': 'application/octet-stream', 'Prediction-Key':'2daa28fe6f654cd1a9e62e4df3242be9'})
        # rospy.loginfo(res.text)
        returned_predictions = process_predictions(res.text)
        processed_image_pub = rospy.Publisher('processed_image', CompressedImage, queue_size=10)
        # print(returned_predictions)
        if len(returned_predictions)>0:
            image_np = get_image(data.data)
            draw_rectangle(returned_predictions, image_np)
            processed_image = prepare_processed_image(data, image_np)
            processed_image_pub.publish(processed_image)
    except OSError:
        rospy.logwarn("Connection error, make sure docker is running and available at address: http://172.17.0.2")
    except:
        rospy.logwarn("Something went wrong")

def predictor():

    rospy.init_node('predictor', anonymous=True)
    rospy.Subscriber('/webcam/image_raw/compressed', CompressedImage, callback, queue_size=1)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    predictor()
