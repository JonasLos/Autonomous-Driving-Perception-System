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
        img = ros_numpy.numpify(msg)
        base64_image = encode_image(img)

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
                                    """,
                        },
                        {
                            "type": "image_url",
                            "image_url": {
                                "url": f"data:image/jpeg;base64,{base64_image}"
                            },
                        },
                    ],
                }
            ],
        )
        print(response.choices[0].message.content)
        print("Time for inference", time.time() - a)
    except Exception as e:
        print(f"Error processing image: {e}")


if __name__ == "__main__":
    print("Initializing ROS node...")
    rospy.init_node("image_listener")
    print("Subscribing to ROS camera topic...")
    rospy.Subscriber(CAMERA_TOPIC, Image, image_callback)
    print("Listening for images...")
    rospy.spin()