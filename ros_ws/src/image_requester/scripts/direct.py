#!/usr/bin/env python3
# Software License Agreement (BSD License)

from turtle import color
import numpy as np
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from common import *
import cv2
import io
from PIL import Image
# Imports for prediction
from predict import initialize, predict_image


font = cv2.FONT_HERSHEY_SIMPLEX



def callback(data):
    try:
        imageData = io.BytesIO(data.data)
        img = Image.open(imageData)
        res = predict_image(img)
        # print(res)
        res = res.replace("\'", "\"")

        returned_predictions = process_predictions(res)
        processed_image_pub = rospy.Publisher('processed_image', CompressedImage, queue_size=10)
        # print(data.format)
        if len(returned_predictions)>0:
            np_array = np.frombuffer(data.data, np.uint8)
            image_np = cv2.imdecode(np_array, cv2.IMREAD_COLOR)
            for p in returned_predictions:
                start_point, end_point = find_points(p)
                colour = get_colour(p)
                cv2.rectangle(image_np,start_point,end_point,colour,2)
                mid_point = find_midpoint(start_point, end_point)
                name = p["tagName"] + " %"+str(round(100*p["probability"]))
                cv2.putText(image_np, name,mid_point,font,2,colour,2,cv2.LINE_AA)
            processed_image = CompressedImage()
            processed_image.header = data.header
            processed_image.header.stamp = rospy.Time.now()
            processed_image.data = np.array(cv2.imencode('.jpg', image_np)[1]).tobytes()
            processed_image.format = "jpeg"
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
    initialize()
    predictor()
