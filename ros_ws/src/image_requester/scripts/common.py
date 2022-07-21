import json
import rospy
from std_msgs.msg import String
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage

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

def get_colour(p):
    if p["tagName"]=="Mohsen":
        return (0,255,0)
    elif p["tagName"]=="Ali":
        return (0,0,255)
    else:
        return (255,0,0)

def draw_rectangle(returned_predictions, image_np):
    for p in returned_predictions:
                start_point, end_point = find_points(p)
                colour = get_colour(p)
                cv2.rectangle(image_np,start_point,end_point,colour,2)
                mid_point = find_midpoint(start_point, end_point)
                name = p["tagName"] + " %"+str(round(100*p["probability"]))
                cv2.putText(image_np, name,mid_point,font,2,colour,2,cv2.LINE_AA)

def get_image(data):
     np_array = np.frombuffer(data, np.uint8)
     image_np = cv2.imdecode(np_array, cv2.IMREAD_COLOR)
     return image_np

def prepare_processed_image(data, image_np):
    processed_image = CompressedImage()
    processed_image.header = data.header
    processed_image.header.stamp = rospy.Time.now()
    processed_image.data = np.array(cv2.imencode('.jpg', image_np)[1]).tobytes()
    processed_image.format = "jpeg"
    return processed_image
