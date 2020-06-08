import numpy as np
from rostest.msg import image
import cv2


class ImageConverter:
    def cv2_to_msg(self, img: np.ndarray) -> image:
        # ROS messages need one-dimensional lists as arguments
        return image(img.tobytes(), img.shape[0], img.shape[1])
    def msg_to_cv2(self, msg: image) -> np.ndarray:
        # Convert the channels from bytes to uint8 arrays and reconstruct the shape
        return np.frombuffer(msg.data, "uint8").reshape(msg.height, msg.width, 3)
