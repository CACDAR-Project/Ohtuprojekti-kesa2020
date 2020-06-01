#!/usr/bin/env python3.7
import cv2 as cv
import tensorflow as tf
import numpy as np
from typing import List
import rospy
from rostest.msg import observation, boundingbox
from std_msgs.msg import String

# https://github.com/opencv/opencv/wiki/TensorFlow-Object-Detection-API
tf.compat.v1.disable_v2_behavior()


class ObjectDetector:
    labels: List[str]

    def __init__(self, model_path: str, labels_path: str):
        self.load_model(model_path)
        self.load_labels(labels_path)

    def load_labels(self, label_path: str):
        self.labels = []
        with open(label_path) as f:
            while True:
                line = f.readline().rstrip()
                if len(line) == 0:  # File ends
                    break
                self.labels.append(line)

    def load_model(self, model_path: str):
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

        boxes = self.interpreter.get_tensor(self.output_details[0]['index'])[0]
        classes = self.interpreter.get_tensor(
            self.output_details[1]['index'])[0]
        scores = self.interpreter.get_tensor(
            self.output_details[2]['index'])[0]

        # Visualize detected bounding boxes.
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

    def print_input(self, input):
        print("Recieved input from another node: " + input.data)

    def run(self, showgui: bool):
        pub = rospy.Publisher("observations", observation, queue_size=50)
        input_sub = rospy.Subscriber("inputs", String, self.print_input)
        cam = cv.VideoCapture(
            "test.mp4")  # Can be replaced with camera id, path to camera etc.
        while cam.grab():
            # Read and preprocess an image.
            img = cam.retrieve()[1]
            height = img.shape[0]
            width = img.shape[1]
            for detection in self.detect(img):
                pub.publish(
                    observation(
                        detection["class_id"], detection["label"],
                        detection["score"],
                        boundingbox(detection["bbox"]["top"],
                                    detection["bbox"]["right"],
                                    detection["bbox"]["bottom"],
                                    detection["bbox"]["left"])))
                if showgui:
                    # Draw boxes around objects
                    top = detection["bbox"]["top"] * height
                    left = detection["bbox"]["left"] * width
                    right = detection["bbox"]["right"] * width
                    bottom = detection["bbox"]["bottom"] * height
                    cv.putText(
                        img,
                        "{} score: {}".format(detection["label"],
                                              round(detection["score"], 3)),
                        (int(left), int(top - 5)), cv.QT_FONT_NORMAL, 1,
                        (255, 0, 255), 1, cv.LINE_AA)
                    cv.rectangle(img, (int(left), int(top)),
                                 (int(right), int(bottom)), (125, 255, 51),
                                 thickness=2)

            # Display the result
            if showgui:
                cv.imshow('img', img)
                cv.waitKey(1)

    def close(self):
        self.sess.close()


if __name__ == "__main__":
    rospy.init_node("Testnode")
    detector = ObjectDetector("ssd_mobilenet_v1_1_metadata_1.tflite",
                              "mscoco_complete_labels")

    detector.run(False)
    #detector.run(True)
    ros.spin()
