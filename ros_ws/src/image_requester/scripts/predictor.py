#!/usr/bin/env python3
# Software License Agreement (BSD License)

import rospy
from sensor_msgs.msg import CompressedImage
import requests
import cv2
from common import *
from threading import Thread
from queue import Queue

font = cv2.FONT_HERSHEY_SIMPLEX

q = Queue(maxsize=1)

def process_callback(q: Queue):
    while True:
        data = q.get()
        res = requests.post(url='http://172.17.0.2/image',
                        data=data.data,
                        headers={'Content-Type': 'application/octet-stream', 'Prediction-Key':'2daa28fe6f654cd1a9e62e4df3242be9'})
        returned_predictions = process_predictions(res.text)
        processed_image_pub = rospy.Publisher('processed_image', CompressedImage, queue_size=10)
        if len(returned_predictions)>0:
            image_np = get_image(data.data)
            draw_rectangle(returned_predictions, image_np)
            processed_image = prepare_processed_image(data, image_np)
            processed_image_pub.publish(processed_image)
    


def callback(data):
    global q
    if not q.full():
        q.put(data)

def predictor():
    rospy.init_node('predictor')
    callback_processor = Thread(target=process_callback,args=(q,), daemon=True)
    callback_processor.start()
    rospy.Subscriber('/webcam/image_raw/compressed', CompressedImage, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    try:
        predictor()
    except KeyboardInterrupt:
        rospy.INFO("Exiting node")
        exit(0)
