import cv2
import numpy as np

from typing import Union
from application.conf.configuration import Configuration


class Camera():
    def __init__(self, cam_id: Union[int] = None) -> None:
        '''
        Camera ID defined at construction. If ID is -1, try to find a working camera.
        If ID is None use the default ID from app configuration.
        '''
        if not cam_id:
            cam_id = Configuration.get_instance().settings['VIDEO_ID']
        elif cam_id == -1:
            cam_id = self.camera_scan()
        elif not isinstance(cam_id, int):
            raise TypeError(
                'Error trying to initialize camera. cam_id must be None or integer.'
            )

        self.set_camera_id(cam_id)
        self.shape = Configuration.get_instance().settings['DEF_SHAPE']

    def camera_scan(self) -> int:
        '''
        Go trough indices from 0 until a working videostream is opened.
        Return the camera ID.
        '''
        for i in range(10):
            cap = cv2.VideoCapture(i)
            if cap.isOpened():
                return i
        return 0

    def set_camera_id(self, cam_id: int) -> None:
        self.camera_id = cam_id
        self.cap = cv2.VideoCapture(self.camera_id)

    def set_shape(width: int, height: int) -> None:
        self.shape = (height, width, 3)

    def frameRGB(self) -> np.ndarray:
        '''
        Return the newest fram from the camera as a 3 dimensional nupmy array.
        '''

        ret, frame = self.cap.read()
        # if ret is False, no image was captured and black image is returned
        if not ret:
            return np.zeros(shape=self.shape).astype(np.uint8)
        return frame
