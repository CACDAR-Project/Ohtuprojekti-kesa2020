import numpy as np
from konenako.msg import image
import cv2


class ImageConverter:
    def cv2_to_msg(self, img: np.ndarray) -> image:
        # ROS messages need one-dimensional lists as arguments
        red = img[:, :, 0].flatten().tolist()
        green = img[:, :, 1].flatten().tolist()
        blue = img[:, :, 2].flatten().tolist()
        msg = image(red, green, blue, img.shape[0], img.shape[1])
        return msg

    def msg_to_cv2(self, msg: image) -> np.ndarray:
        # Convert the channels from bytes to uint8 arrays and reconstruct the shape
        red2 = np.frombuffer(msg.red, dtype=np.uint8)
        red2 = np.reshape(red2, newshape=(msg.height, msg.width))
        green2 = np.frombuffer(msg.green, dtype=np.uint8)
        green2 = np.reshape(green2, newshape=(msg.height, msg.width))
        blue2 = np.frombuffer(msg.blue, dtype=np.uint8)
        blue2 = np.reshape(blue2, newshape=(msg.height, msg.width))

        # Stack the channels to a single RGB image
        rgb = np.dstack((red2, green2, blue2))

        return rgb
