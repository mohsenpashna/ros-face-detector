#!/usr/bin/env python3
# Software License Agreement (BSD License)

import numpy as np
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
import requests
import json
import cv2


font = cv2.FONT_HERSHEY_SIMPLEX


def find_points(p):
    img_width = 640
    img_height = 480
    top = round(p["boundingBox"]["top"]*img_height)
    left =  round(p["boundingBox"]["left"]*img_width)
    height = round(p["boundingBox"]["height"]*img_height)
    width = round(p["boundingBox"]["width"]*img_width)
    start_point = (top,left)
    end_point = (top+height, left+width)
    return start_point,end_point

def find_midpoint(start_point, end_point):
    return(round((start_point[0])),round((start_point[1]+end_point[1])*0.5))


def process_predictions(raw_predictions):
    jpred = json.loads(raw_predictions)
    preds = jpred["predictions"]
    returned_predictions = []
    for p in preds:
        if p["probability"]>0.9:
            # print(p)
            returned_predictions.append(p)
            pub = rospy.Publisher('predictions', String, queue_size=10)
            pub.publish(str(p))
    return returned_predictions



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
            np_array = np.frombuffer(data.data, np.uint8)
            image_np = cv2.imdecode(np_array, cv2.IMREAD_COLOR)
            for p in returned_predictions:
                # print(p)
                start_point, end_point = find_points(p)
                cv2.rectangle(image_np,start_point,end_point,(0,0,255),2)
                mid_point = find_midpoint(start_point, end_point)
                name = p["tagName"] + " %"+str(round(100*p["probability"]))
                cv2.putText(image_np, name,mid_point,font,2,(0,0,255),2,cv2.LINE_AA)
            processed_image = CompressedImage()
            processed_image.header = data.header
            processed_image.header.stamp = rospy.Time.now()
            processed_image.data = np.array(cv2.imencode('.jpg', image_np)[1]).tobytes()
            processed_image.format = data.format
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
