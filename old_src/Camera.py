import cv2
import numpy as np


class Camera:
    def __init__(self, cam_id):
        """Video dimensions and camera ID defined at construction."""
        self.camera_id = cam_id
        self.cap = cv2.VideoCapture(self.camera_id)

    def frameRGB(self):
        """Return the newest frame from the camera as a 3 dimensional numpy array."""
        ret, frame = self.cap.read()
        # if ret is False, no image was captured and black image is returned
        if ret == False:
            return np.zeros(shape=(300, 300, 3)).astype(np.uint8)

        return frame
