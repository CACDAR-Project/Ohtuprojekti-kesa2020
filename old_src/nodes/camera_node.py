import cv2
import numpy as np
import camera_node_interface


class Camera(camera_node_interface.CameraNodeInterface):
    def __init__(self, cam_id: int):
        """
        Camera ID defined at construction. If ID is -1, try to find a working camera.
        """
        if (cam_id == -1):
            cam_id = self.camera_scan()
        self.set_camera_id(cam_id)

    def camera_scan(self) -> int:
        """
        Go through indices from 0 until a working videostream is opened. 
        Return the camera ID.
        """
        for i in range(10):
            cap = cv2.VideoCapture(i)
            if cap.isOpened():
                return i
        return 0

    def set_camera_id(self, cam_id: int):
        self.camera_id = cam_id
        self.cap = cv2.VideoCapture(self.camera_id)

    def frameRGB(self) -> np.ndarray:
        """
        Return the newest frame from the camera as a 3 dimensional numpy array.
            """
        ret, frame = self.cap.read()
        # if ret is False, no image was captured and black image is returned
        if ret == False:
            return np.zeros(shape=(300, 300, 3)).astype(np.uint8)

        return frame
