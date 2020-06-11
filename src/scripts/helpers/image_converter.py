## @package helpers
#  Contains functions for converting between numpy array image
#  to project's ROS image message.

import numpy as np
from konenako.msg import image
import cv2

## Convert numpy array image to ROS image message
#  @param image as numpy array
#  @return ROS image message
def cv2_to_msg(img: np.ndarray) -> image:
    # ROS messages need one-dimensional lists as arguments
    return image(img.tobytes(), img.shape[0], img.shape[1])

## Convert ROS image message to numpy array image
#  @param ROS image message
#  @return image as numpy array
def msg_to_cv2(msg: image) -> np.ndarray:
    # Convert the channels from bytes to uint8 arrays and reconstruct the shape
    return np.frombuffer(msg.data, "uint8").reshape(msg.height, msg.width, 3)
