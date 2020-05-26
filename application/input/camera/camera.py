import cv2
import numpy as np

from typing import Union, Tuple
from application.conf.configuration import Configuration


class Camera():
    def __init__(self, cam_id: Union[int] = None) -> None:
        '''
        Camera ID defined at construction. If ID is -1, try to find a working camera.
        If ID is None use the default ID from app configuration.
        '''
        if not cam_id:
            cam_id = Configuration.get_instance().settings['VIDEO_ID']

        #if isinstance(cam_id, str) # This can be a path to a videofile.
        if not isinstance(cam_id, int):
            raise TypeError(
                'Error trying to initialize camera. cam_id must be None or integer.'
            )

        if cam_id == -1:
            cam_id = self.camera_scan()

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
        if not self.cap.isOpened():
            raise RuntimeError(
                f'Could not load videostream with id: {cam_id}, exiting..')

    def set_shape(self, width: int, height: int) -> None:
        self.shape = (height, width, 3)

    def get_fps(self):
        return self.cap.get(5)

    def get_size(self) -> Tuple[int, int]:
        'Return video width x height as a tuple'
        return (int(self.cap.get(3)), int(self.cap.get(4)))

    def set_size(self, width: int, height: int) -> None:
        'Set the size of the video'
        self.cap.set(3, width)
        self.cap.set(4, height)

    def print_video_info(self):
        '''Print all video properties
        https://docs.opencv.org/2.4/modules/highgui/doc/reading_and_writing_images_and_video.html#videocapture-get
        '''
        for i in range(19):
            print(self.cap.get(i))

    def frameBGR(self) -> np.ndarray:
        '''
        Return the newest fram from the camera as a 3 dimensional nupmy array.
        '''

        ret, frame = self.cap.read()

        # if ret is False, no image was captured and black image is returned
        if not ret:
            return np.zeros(shape=self.shape).astype(np.uint8)
        return frame

    def frames(self) -> Tuple[np.ndarray, np.ndarray]:
        '''
        Return a tuple containing the newest frame in BGR as well as the same frame in gray
        '''
        frame = self.frameBGR()
        return frame, cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
