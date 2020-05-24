from itertools import cycle
from typing import Tuple, Iterable

import cv2
import numpy as np

from application.conf.configuration import Configuration


class Screen():
    '''Holds one frame and draws the frame with cv2'''
    def __init__(self, gray=False):
        '''Constructor. Takes one argument that when passed True sets the output color to gray.
        Default value is False -> Color output'''
        self._reset_colors()
        self.frame_in = None
        self.frame_out = None
        self.gray_bool = gray

        self.rectangles = None
        self.shapes = None
        self.texts = None

    def __del__(self):
        cv2.destroyAllWindows()

    def _reset_colors(self):
        '''Use itertools.cycle to iterate between colors, this method resets the iterators.
        CV2 uses BGR colors instead of normal RGB.
        '''
        self.colors_blue = cycle(Configuration.get_instance().colors['blue'])
        self.colors_green = cycle(Configuration.get_instance().colors['green'])
        self.colors_red = cycle(Configuration.get_instance().colors['red'])

    def set_frame(self, frame: np.ndarray) -> None:
        '''Takes one argument, the frame to draw as an np.ndarray in cv2-format'''
        self.frame_in = frame
        self.rectangles = None
        self.shapes = None
        self.texts = None
        self._reset_colors()  # Reset color iterators for every frame

    def add_rectangles(
            self, rectangles: Iterable[Tuple[Tuple[int, int],
                                             Tuple[int, int]]]) -> None:
        '''Adds all rectangles from the rectangles iterable'''
        if self.rectangles:
            self.rectangles.extend(rectangles)
        else:
            self.rectangles = rectangles

    def add_rectangle(self, top_left: Tuple[int, int],
                      bottom_right: Tuple[int, int]) -> None:
        'Adds an rectangle to the frame, takes two tuples each containing x,y coords'
        if not self.rectangles:
            self.rectangles = list()

        self.rectangles.append((top_left, bottom_right))

    #def add_box(self, x, y, width, height) -> None:
    #    self.boxes.append( ( (x, y), (x+width, y+height)) )

    def add_polygons(self, points: np.array) -> None:
        'Adds the np.array containing points for n polygons to be drawed onto the frame'
        if self.shapes is None:
            self.shapes = points
        else:
            print(
                'Coder was too lazy to extend numpy array... POLYGONS DROPPED!'
            )

    def add_text(self, text_with_coords: Tuple[str, Tuple[int, int]]) -> None:
        'Adds the text passed as parameter on this frame on the coords passed'
        #self.texts.append((text, (x, y)))
        if not self.texts:
            self.texts = list()
        self.texts.append(text_with_coords)

    def add_texts(self, texts) -> None:
        if self.texts:
            self.texts.extend(texts)
        else:
            self.texts = texts

    def toggle_gray(self) -> None:
        'Toggles the output to be either in color or gray'
        self.gray_bool = not self.gray_bool

    def draw(self):
        if self.gray_bool:
            self.frame_out = cv2.cvtColor(self.frame_in, cv2.COLOR_BGR2GRAY)
        else:
            self.frame_out = self.frame_in

        if self.rectangles:
            for b in self.rectangles:
                cv2.rectangle(self.frame_out, b[0], b[1],
                              next(self.colors_blue), 2)

        if self.shapes is not None:
            for p in self.shapes:
                cv2.polylines(self.frame_out, [p], True, next(self.colors_red),
                              2)

        if self.texts:
            offset_loc = lambda t: (t[0] + 3, t[
                1] + 18)  # Move text coords so text is inside rectangle
            for t in self.texts:
                cv2.putText(self.frame_out, t[0], offset_loc(t[1]),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

        cv2.imshow('VIDEO OUT', self.frame_out)
