import cv2 as cv
import tensorflow as tf
from typing import List

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
        with tf.compat.v1.gfile.GFile(model_path, 'rb') as f:
            self.graph_def = tf.compat.v1.GraphDef()
            self.graph_def.ParseFromString(f.read())
        self.sess = tf.compat.v1.Session()
        self.sess.graph.as_default()
        tf.import_graph_def(self.graph_def, name='')

    # Input BGR from OpenCV
    def detect(self, img):
        inp = cv.resize(img, (300, 300))
        inp = inp[:, :, [2, 1, 0]]  # BGR2RGB

        # Run the model
        out = self.sess.run([
            self.sess.graph.get_tensor_by_name('num_detections:0'),
            self.sess.graph.get_tensor_by_name('detection_scores:0'),
            self.sess.graph.get_tensor_by_name('detection_boxes:0'),
            self.sess.graph.get_tensor_by_name('detection_classes:0')
        ],
                            feed_dict={
                                'image_tensor:0':
                                inp.reshape(1, inp.shape[0], inp.shape[1], 3)
                            })

        # Visualize detected bounding boxes.
        num_detections = int(out[0][0])
        detections = []
        for i in range(num_detections):
            classId = int(out[3][0][i])
            score = float(out[1][0][i])
            bbox = [float(v) for v in out[2][0][i]]
            if score > 0.3:
                detections.append({
                    "classId": classId,
                    "label": self.labels[int(classId)],
                    "score": score,
                    "bbox": {
                        "top": bbox[0],
                        "right": bbox[3],
                        "bottom": bbox[2],
                        "left": bbox[1]
                    }
                })
        return detections

    def run(self):
        cam = cv.VideoCapture(
            "test.mp4")  # Can be replaced with camera id, path to camera etc.
        while cam.grab():
            # Read and preprocess an image.
            img = cam.retrieve()[1]
            height = img.shape[0]
            width = img.shape[1]
            for detection in self.detect(img):
                # Draw boxes around objects and display the result
                top = detection["bbox"]["top"] * height
                left = detection["bbox"]["left"] * width
                right = detection["bbox"]["right"] * width
                bottom = detection["bbox"]["bottom"] * height
                cv.putText(
                    img, "{} score: {}".format(detection["label"],
                                               round(detection["score"], 3)),
                    (int(left), int(top - 5)), cv.QT_FONT_NORMAL, 1,
                    (255, 0, 255), 1, cv.LINE_AA)
                cv.rectangle(img, (int(left), int(top)),
                             (int(right), int(bottom)), (125, 255, 51),
                             thickness=2)
            cv.imshow('img', img)
            cv.waitKey(1)
            print(detection)

    def close(self):
        self.sess.close()


if __name__ == "__main__":
    detector = ObjectDetector(
        "ssdlite_mobilenet_v2_coco_2018_05_09/frozen_inference_graph.pb",
        "mscoco_complete_labels")
    detector.run()
