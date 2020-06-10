import numpy as np
from konenako.msg import image
import cv2


def cv2_to_msg(img: np.ndarray) -> image:
    # ROS messages need one-dimensional lists as arguments
    return image(img.tobytes(), img.shape[0], img.shape[1])


def msg_to_cv2(msg: image) -> np.ndarray:
    # Convert the channels from bytes to uint8 arrays and reconstruct the shape
    return np.frombuffer(msg.data, "uint8").reshape(msg.height, msg.width, 3)
