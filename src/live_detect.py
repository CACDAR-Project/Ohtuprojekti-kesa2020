# image downloaded from: https://www.apa.org/monitor/2020/04/nurtured-nature
# code from https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_gui/py_image_display/py_image_display.html#goals and https://www.tensorflow.org/lite/???

import cv2
import numpy as np
from matplotlib import pyplot as plt
import argparse
import tensorflow as tf
import Camera

DEFAULT_VIDEO_ID = 0
DEFAULT_MODEL = 'detect.tflite'
DEFAULT_LABELS = 'labelmap.txt'


def show_image():
    '''
    Load and convert an image from disk to grayscale. draw a line and show the result. OpenCV with pyplot
    '''
    img = cv2.imread('images/nature.png', cv2.IMREAD_GRAYSCALE)
    #cv2.imshow('image', img)
    #cv2.waitKey(0)
    #cv2.destroyAllWindows()

    plt.imshow(img, cmap='gray', interpolation='bicubic')
    plt.xticks([])
    plt.yticks([])
    plt.plot([200, 300, 400], [100, 200, 300], 'c', linewidth=5)
    plt.show()

    #cv2.imwrite('naturegray.png', img) #save image


def show_video():

    cap = cv2.VideoCapture(args.camera_id)

    while True:
        # ret is boolean indicating if we have any image returnedor not.
        ret, frame = cap.read()
        # convert frame to grayscale. Opencv uses BGR-colors as oposed to RGB
        #gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        cv2.imshow('frame', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


def detect_objects():
    interpreter = tf.lite.Interpreter(model_path=args.model_file)
    interpreter.allocate_tensors()

    input_details = interpreter.get_input_details()
    output_details = interpreter.get_output_details()

    # check the type of the input tensor
    floating_model = input_details[0]['dtype'] == np.float32
    # NxHxWxC, H:1, W:2
    height = input_details[0]['shape'][1]
    width = input_details[0]['shape'][2]

    cam = Camera.Camera(int(args.camera_id))
    labels = load_labels(args.label_file)

    while True:
        # Get frame from camera.
        img = cam.frameRGB()

        # Display the frame.
        cv2.imshow('frame', img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        # Resize to the size the model is trained on.
        img = np.resize(img, (width, height, 3))

        # Add N dim.
        input_data = np.expand_dims(img, axis=0)

        if floating_model:
            input_data = (np.float32(input_data) -
                          args.input_mean) / args.input_std

        interpreter.set_tensor(input_details[0]['index'], input_data)
        interpreter.invoke()

        output_data = interpreter.get_tensor(output_details[0]['index'])
        results = np.squeeze(output_data)

        classes = interpreter.get_tensor(
            output_details[1]['index']).astype(int)
        scores = interpreter.get_tensor(output_details[2]['index'])
        print("Classes and scores:")
        for i in range(6):
            print(f'{labels[classes[0][i]]:10} {scores[0][i]}')


def load_labels(filename):
    with open(filename, 'r') as f:
        return [line.strip() for line in f.readlines()]


def record_video():
    '''
    Show video and also save as xvid-encoded
    '''
    cap = cv2.VideoCapture(VIDEO_DEVICE_ID)
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    out = cv2.VideoWriter('output.avi', fourcc, 20.0, (640, 480))

    while True:
        ret, frame = cap.read()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        out.write(frame)  #in colors
        cv2.imshow('frame', gray)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    out.release()
    cv2.destroyAllWindows()


def read_img():
    '''
    OpenCV
    '''
    img = cv2.imread('images/nature.png', cv2.IMREAD_COLOR)

    #x,y, start coords, end coords, color (bgr), line thickness
    cv2.line(img, (0, 0), (130, 130), (255, 255, 255), 10)

    #top left, bottom right
    cv2.rectangle(img, (135, 45), (230, 130), (0, 255, 0), 5)
    #center of circle, radius, color and linewidth, negative fills the circle
    cv2.circle(img, (350, 250), 55, (0, 0, 255), -1)

    # Polygons
    # List of points
    #pts = np.array([[10,5], [20,30], [70,20], [50,10]], np.int32)
    pts = np.array([[50, 30], [150, 30], [100, 90]], np.int32)
    #pts = pts.reshape((-1,1,2)) # correct shape of points array
    cv2.polylines(img, [pts], True, (0, 255, 255), 3)

    #Writing text
    font = cv2.FONT_HERSHEY_SIMPLEX
    #place, size, color, text thickness, antialiazing
    cv2.putText(img, 'OpenCV testing', (0, 130), font, 3, (200, 255, 255), 2,
                cv2.LINE_AA)
    cv2.imshow('image', img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


def main():
    # exit all methods with pressing q
    # show_image()
    #show_video()
    #record_video()
    #read_img()
    detect_objects()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-m',
                        '--model_file',
                        default=DEFAULT_MODEL,
                        help='.tflite model to be executed')
    parser.add_argument('-l',
                        '--label_file',
                        default=DEFAULT_LABELS,
                        help='name of file containing labels')
    parser.add_argument('--input_mean',
                        default=127.5,
                        type=float,
                        help='input_mean')
    parser.add_argument('--input_std',
                        default=127.5,
                        type=float,
                        help='input standard deviation')
    parser.add_argument('-c',
                        '--camera_id',
                        default=DEFAULT_VIDEO_ID,
                        help='number of the a webcamera')

    args = parser.parse_args()

    main()
