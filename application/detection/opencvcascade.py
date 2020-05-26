import cv2
import numpy as np

from typing import Tuple, Union


class OpenCVCascade():
    def __init__(self,
                 cascade_classifier: str,
                 scale_factor: float = 1.3,
                 min_neighbors: int = 5) -> None:
        self.scale_factor = scale_factor
        self.min_neighbors = min_neighbors
        self.detections = tuple(
        )  # self.cascade will return empty tuple if nothing was detected, np.ndarray otherwise
        self.cascade = cv2.CascadeClassifier(cascade_classifier)

    def scan_frame(self, frame: np.ndarray) -> None:
        '''Takes one frame and applies the cascade classifier to it'''
        self.detections = self.cascade.detectMultiScale(
            frame,
            scaleFactor=self.scale_factor,
            minNeighbors=self.min_neighbors)

    def get_detections(self) -> Union[Tuple, np.ndarray]:
        return self.detections

    def get_center_points(self) -> Tuple[Tuple[int, int]]:
        '''Retuns the center coordinates for every detection'''
        return tuple(
            map(lambda c: (c[0] + c[2] // 2, c[1] + c[3] // 2),
                self.detections))

    def get_center_points_with_radius(
            self) -> Tuple[Tuple[Tuple[int, int], int]]:
        '''Returns the center coordinates with the radius for a circle as a 3d tuple.
        (((x,y), r), (..), ..)
        '''
        return tuple(
            map(
                lambda c: ((c[0] + c[2] // 2, c[1] + c[3] // 2),
                           (c[2] + c[3]) // 4), self.detections))

    def get_rectangles_coords(
            self) -> Tuple[Tuple[Tuple[int, int], Tuple[int, int]]]:
        '''Returns a 3d tuple containing one tuple for every code coords in the form that
        opencv can draw rectangles: (((xtopleft,ytopleft),(xbottomright,ybottomright)), ((..),(..)). ..)
        '''
        return tuple(
            map(lambda c: ((c[0], c[1]), (c[0] + c[2], c[1] + c[3])),
                self.detections))

    def get_detections_amount(self) -> int:
        #return self.faces.shape[0]
        return len(self.detections)
