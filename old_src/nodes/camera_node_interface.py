import numpy as np


class CameraNodeInterface:
    def frameRGB(self) -> np.ndarray:
        pass
