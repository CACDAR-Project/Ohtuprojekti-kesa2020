import numpy as np
from typing import List


class DetectionNodeInterface:
    def process_frame(self, frame: np.ndarray):
        pass

    def last_result(self) -> List:
        pass
