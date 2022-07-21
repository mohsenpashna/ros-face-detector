import json
import rospy
from std_msgs.msg import String

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
