import base64
import time

import cv2
import ros_numpy
import rospy
from cv_bridge import CvBridge
from openai import OpenAI
from sensor_msgs.msg import Image

from configs import CAMERA_TOPIC

client = OpenAI(api_key="")

bridge = CvBridge()
last_request_time = 0
interval = 0  # seconds

def encode_image(cv_image):
    _, buffer = cv2.imencode(".jpg", cv_image)
    return base64.b64encode(buffer).decode("utf-8")

def image_callback(msg):
    global last_request_time
    current_time = time.time()
    if current_time - last_request_time < interval:
        # print("Skipping frame: waiting for next interval")
        return
    last_request_time = current_time
    
    try:
        # print("Received image from ROS topic")
        img = ros_numpy.numpify(msg)
        # img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        # cv2.imshow("Image Sent to OpenAI", img)
        # cv2.waitKey(1)
        # print("Converted ROS image to OpenCV format")
        base64_image = encode_image(img)
        # print("Encoded image to Base64")

        a = time.time()
        response = client.chat.completions.create(
            model="gpt-4o-mini",
            messages=[
                {
                    "role": "user",
                    "content": [
                        {
                            "type": "text",
                            "text": """
                                    I am working on an autonomous driving project and need to determine the coefficient 
                                    of friction of the surface my vehicle is driving on. I will provide you with an 
                                    image from the vehicle's camera. Classify the road surface 
                                    type from the following categories:

                                    - Dry asphalt or concrete 
                                    - Wet asphalt or concrete 
                                    - Gravel
                                    - Earth road (dry)  
                                    - Earth road (wet)  
                                    - Snow (hard-packed)  
                                    - Ice  
                                    Just give the output as one of the above classes and nothing else.
                                    """
                        },
                        {
                            "type": "image_url",
                            "image_url": {"url": f"data:image/jpeg;base64,{base64_image}"},
                        },
                    ],
                }
            ],
        )
        # print("Received response from GPT API:")
        print(response.choices[0].message.content)
        print('This is how long it take for the model inference', time.time() - a)
    except Exception as e:
        print(f"Error processing image: {e}")

if __name__ == "__main__":
    print("Initializing ROS node...")
    rospy.init_node("image_listener")
    print("Subscribing to ROS camera topic...")
    rospy.Subscriber(CAMERA_TOPIC, Image, image_callback)
    print("Listening for images...")
    rospy.spin()


prompt1 = '''                  
You are a perception module in an autonomous driving system. Analyze the provided image captured from the forward-facing camera of a vehicle. Your task is to identify key components of the scene and return detailed textual descriptions based on the following instructions:
Driveable Road Area Identification:
Describe the area of the road that appears safe and suitable for driving. Mention the position of lanes, road boundaries, and any visible lane markings.
Tell me the the surface type of the driveable area.
Highlight any obstacles or regions that should be avoided (e.g., curbs, sidewalks, construction zones).
Object Detection and Classification:
Detect and describe objects present in the image. For each object, return:
Object class (type): Identify objects like vehicles (car, truck, bus), pedestrians, cyclists, traffic signs, or other relevant items.
Relative location: Provide a description of where each object is positioned within the scene (e.g., \"a pedestrian on the right sidewalk,\" \"a car directly ahead\").
Relative distance: Indicate proximity using terms like \"near,\" \"far,\" or \"approaching.\"
Traffic Signs and Signals:
Identify and interpret any visible traffic signs or signals, stating their meaning and potential driving actions required (e.g., \"a stop sign ahead,\" \"green traffic light\").
Safe Driving Recommendations:
Based on your analysis, provide a recommendation for the next safe driving action (e.g., \"reduce speed for pedestrian crossing\" or \"proceed straight within the lane\").
Input:
A single image from a forward-facing vehicle camera sensor.
Output:
A comprehensive text-based description, structured as:
Driveable Road Area: Description of the safe road area.
Detected Objects: List of objects with class labels, relative positions, and proximities.
Traffic Signage or Signals: Any signs or signals and their driving implications.
Driving Recommendation: Suggested action based on scene analysis.
'''