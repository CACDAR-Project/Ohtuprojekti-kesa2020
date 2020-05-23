import numpy as np
import tensorflow as tf
from typing import List
import detection_node_interface

DEFAULT_VIDEO_ID = 0
DEFAULT_MODEL = 'detect.tflite'
DEFAULT_LABELS = 'labelmap.txt'


class ObjectDetectionNode(detection_node_interface.DetectionNodeInterface):
    def __init__(self, model_file: str, label_file: str):
        self.set_model(model_file, label_file)

    def set_model(self, model_file: str, label_file: str):
        """
        Define the model and label filename to use for object detection.
        """
        self.label_file = label_file
        self.interpreter = tf.lite.Interpreter(model_file)
        self.interpreter.allocate_tensors()
        self.input_details = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()
        # check the type of the input tensor
        self.floating_model = self.input_details[0]['dtype'] == np.float32
        # NxHxWxC, H:1, W:2
        self.height = self.input_details[0]['shape'][1]
        self.width = self.input_details[0]['shape'][2]

    def process_frame(self, frame: np.ndarray):
        """
        Return a list of object detection results as defined on 
        https://www.tensorflow.org/lite/models/object_detection/overview.
        """
        # Resize to the size the model is trained on.
        frame = np.resize(frame, (self.width, self.height, 3))

        # Add N dim.
        input_data = np.expand_dims(frame, axis=0)

        if self.floating_model:
            input_data = (np.float32(input_data) -
                          args.input_mean) / args.input_std

        # Set the value of the input tensor.
        self.interpreter.set_tensor(self.input_details[0]['index'], input_data)
        self.interpreter.invoke()

        self.output_data = []
        for i in range(4):
            self.output_data.append(
                self.interpreter.get_tensor(self.output_details[i]['index']))

        return self.output_data

    def get_labels(self) -> List[str]:
        with open(self.label_file, 'r') as f:
            return [line.strip() for line in f.readlines()]

    def last_result(self):
        return self.output_data
