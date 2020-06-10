import cv2 as cv
import tensorflow as tf
import numpy as np
from typing import List


class ObjectDetector:
    labels: List[str]
    interpreter: tf.lite.Interpreter

    def __init__(self, model_path: str, labels_path: str):
        self.load_model(model_path)
        self.load_labels(labels_path)

    def load_labels(self, label_path: str) -> None:
        self.labels = []
        with open(label_path) as f:
            while True:
                line = f.readline().rstrip()
                if len(line) == 0:  # File ends
                    break
                self.labels.append(line)

    def load_model(self, model_path: str) -> None:
        self.interpreter = tf.lite.Interpreter(model_path)
        self.interpreter.allocate_tensors()
        self.input_details = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()

    # Input BGR from OpenCV
    def detect(self, img):
        # Prepare the input
        inp = cv.resize(img, (self.input_details[0]['shape'][1],
                              self.input_details[0]['shape'][2]))
        inp = inp[:, :, [2, 1, 0]]  # BGR2RGB
        inp = np.expand_dims(inp, axis=0)

        # Run the model
        self.interpreter.set_tensor(self.input_details[0]['index'], inp)
        self.interpreter.invoke()

        # Read results from tensors
        boxes = self.interpreter.get_tensor(self.output_details[0]['index'])[0]
        classes = self.interpreter.get_tensor(
            self.output_details[1]['index'])[0]
        scores = self.interpreter.get_tensor(
            self.output_details[2]['index'])[0]

        detections = []
        for i in range(len(scores)):
            classId = int(classes[i])
            score = float(scores[i])
            bbox = [float(v) for v in boxes[i]]
            if score > 0.3:
                detections.append({
                    "class_id": classId,
                    "label": self.labels[int(classId) + 1],
                    "score": score,
                    "bbox": {
                        "top": bbox[0],
                        "right": bbox[3],
                        "bottom": bbox[2],
                        "left": bbox[1]
                    }
                })
        return detections
