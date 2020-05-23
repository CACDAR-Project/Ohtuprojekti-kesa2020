import cv2


class Draw():
    def __init__(self, gray=False):
        self.frame_in = None
        self.frame_out = None
        self.gray_bool = gray

    def __del__(self):
        cv2.destroyAllWindows()

    def set_frame(self, frame):
        self.frame_in = frame
        #if self.gray:
        #    self.frame_out = cv2.cvtColor(self.frame_in, cv2.COLOR_BGR2GRAY)
        #else:
        #    self.frame_out = self.frame_in

    def toggle_gray(self):
        self.gray_bool = not self.gray_bool

    def draw(self):
        frame_out = cv2.cvtColor(
            self.frame_in,
            cv2.COLOR_BGR2GRAY) if self.gray_bool else self.frame_in
        cv2.imshow('frame_out', frame_out)
