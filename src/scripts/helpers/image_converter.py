## @package helpers
#  Contains functions for converting between numpy array image
#  to project's ROS image message.

import numpy as np
from konenako.msg import image
import cv2


## Convert numpy array image to ROS image message
#  @param id name to identify the camera from
#  @param counter ID for identifying images
#  @param img image as numpy array
#  @return ROS image message
def cv2_to_msg(id: str, counter: int, img: np.ndarray) -> image:
    # ROS messages need one-dimensional lists as arguments
    return image(id, counter, img.tobytes(), img.shape[0], img.shape[1], img.shape[2])


## Convert ROS image message to numpy array image
#  @param ROS image message
#  @return triple containing camera name, frame counter and the image
def msg_to_cv2(msg: image) -> np.ndarray:
    # Convert the channels from bytes to uint8 arrays and reconstruct the shape
    return (msg.camera_id, msg.image_counter,
            np.frombuffer(msg.data, "uint8").reshape(msg.height, msg.width, msg.channels))
