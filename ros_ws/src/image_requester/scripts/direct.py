#!/usr/bin/env python3
# Software License Agreement (BSD License)

import rospy
from sensor_msgs.msg import CompressedImage
from common import *
import io
from PIL import Image
# Imports for prediction
from predict import initialize, predict_image





def callback(data):
    try:
        imageData = io.BytesIO(data.data)
        img = Image.open(imageData)
        res = predict_image(img)
        res = res.replace("\'", "\"")
        returned_predictions = process_predictions(res)
        processed_image_pub = rospy.Publisher('processed_image', CompressedImage, queue_size=10)
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
    initialize()
    predictor()
