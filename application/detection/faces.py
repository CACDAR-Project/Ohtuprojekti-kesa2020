import cv2
import numpy as np

from typing import Tuple, Union


class FaceDetector():
    def __init__(self, cascade_classifier: str) -> None:
        self.detections = tuple(
        )  # self.face_cascade will return empty tuple if no faces found, np.ndarray otherwise
        self.cascade = cv2.CascadeClassifier(cascade_classifier)

    def scan_frame(self, frame: np.ndarray) -> None:
        self.detections = self.cascade.detectMultiScale(frame, 1.3, 5)

    def get_detections(self) -> Union[np.ndarray]:
        return self.detections

    def get_rectangles_coords(
            self) -> Tuple[Tuple[Tuple[int, int], Tuple[int, int]]]:
        return tuple(
            map(lambda c: ((c[0], c[1]), (c[0] + c[2], c[1] + c[3])),
                self.detections))

    def get_detections_amount(self) -> int:
        #return self.faces.shape[0]
        return len(self.detections)
