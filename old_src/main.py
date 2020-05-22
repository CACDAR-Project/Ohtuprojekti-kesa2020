# code from https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_gui/py_image_display/py_image_display.html#goals

import os
import cv2
import numpy as np
from matplotlib import pyplot as plt

from configuration import Configuration
from tools import filereaderwriter as fileio

path = 'images/face_front.jpg'
#path = '/'.join([conf.paths['PATH_IMAGES'], 'face_front.jpg'])
#path = fileio.build_path(conf.paths['PATH_IMAGES'], 'face_front.jpg')
font = cv2.FONT_HERSHEY_DUPLEX

face_cascade = cv2.CascadeClassifier(
    'data/haarcascade_frontalface_default.xml')
eye_cascade = cv2.CascadeClassifier('data/haarcascade_eye.xml')


def show_image():
    '''
    Load and convert an image from disk to grayscale. draw a line and show the result. OpenCV with pyplot
    '''
    if not check_image(path):
        return

    img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
    # cv2.imshow('image', img)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()

    plt.imshow(img, cmap='gray', interpolation='bicubic')
    plt.xticks([])
    plt.yticks([])
    plt.plot([200, 300, 400], [100, 200, 300], 'c', linewidth=5)
    detect_faces_from_image(path)
    plt.show()

    # cv2.imwrite('naturegray.png', img) #save image


def show_video():
    cap = cv2.VideoCapture(0)
    if not check_video(cap):
        return

    while True:
        ret, frame = cap.read(
        )  # ret is boolean indicating if we have any image returned, will be None if no image is returned
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # convert frame to grayscale. Opencv uses BGR-colors as oposed to RGB

        cv2.imshow('frame', gray)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


def record_video():
    '''
    Show video and also save as xvid-encoded
    '''
    cap = cv2.VideoCapture(0)
    if not check_video(cap):
        return

    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    out = cv2.VideoWriter('output.avi', fourcc, 20.0, (640, 480))

    while True:
        ret, frame = cap.read()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        out.write(frame)  # in colors
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
    image_bool = check_image(path)
    if not image_bool:
        return

    img = cv2.imread(path, cv2.IMREAD_COLOR)

    # x,y, start coords, end coords, color (bgr), line thickness
    cv2.line(img, (0, 0), (130, 130), (255, 255, 255), 10)

    # top left, bottom right
    cv2.rectangle(img, (135, 45), (230, 130), (0, 255, 0), 5)
    # center of circle, radius, color and linewidth, negative fills the circle
    cv2.circle(img, (350, 250), 55, (0, 0, 255), -1)

    # Polygons
    # List of points
    # pts = np.array([[10,5], [20,30], [70,20], [50,10]], np.int32)
    pts = np.array([[50, 30], [150, 30], [100, 90]], np.int32)
    # pts = pts.reshape((-1,1,2)) # correct shape of points array
    cv2.polylines(img, [pts], True, (0, 255, 255), 3)

    # Writing text
    font = cv2.FONT_HERSHEY_SIMPLEX
    # place, size, color, text thickness, antialiazing
    cv2.putText(img, 'OpenCV testing', (0, 130), font, 3, (200, 255, 255), 2,
                cv2.LINE_AA)
    detect_faces_from_image(path)
    cv2.imshow('image', img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


def detect_faces_from_image(search_path):
    found_faces = 0
    img = cv2.imread(search_path)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    faces = face_cascade.detectMultiScale(gray, 1.3, 5)

    for (x, y, w, h) in faces:
        img = cv2.rectangle(img, (x, y), (x + w, y + h), (255, 0, 0), 2)
        roi_gray = gray[y:y + h, x:x + w]
        roi_color = img[y:y + h, x:x + w]
        eyes = eye_cascade.detectMultiScale(roi_gray)
        found_faces += 1
        for (ex, ey, ew, eh) in eyes:
            cv2.rectangle(roi_color, (ex, ey), (ex + ew, ey + eh), (0, 255, 0),
                          2)

    if found_faces > 1:
        info = f'{found_faces} people present'
        cv2.putText(img, info, (0, 20), font, 1, (0, 0, 0), 1, cv2.LINE_4)
    elif found_faces == 1:
        info = f'{found_faces} person present'
        cv2.putText(img, info, (0, 20), font, 1, (0, 0, 0), 1, cv2.LINE_4)
        cv2.imshow('img', img)

    cv2.waitKey(0)
    cv2.destroyAllWindows()


def detect_faces_from_video():
    cap = cv2.VideoCapture(0)
    if not check_video(cap):
        return

    while True:
        ret, img = cap.read()
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        faces = face_cascade.detectMultiScale(gray, 1.3, 5)

        for (x, y, w, h) in faces:
            cv2.rectangle(img, (x, y), (x + w, y + h), (255, 0, 0), 2)
            roi_gray = gray[y:y + h, x:x + w]
            roi_color = img[y:y + h, x:x + w]

            eyes = eye_cascade.detectMultiScale(roi_gray)
            for (ex, ey, ew, eh) in eyes:
                cv2.rectangle(roi_color, (ex, ey), (ex + ew, ey + eh),
                              (0, 255, 0), 2)
        cv2.imshow('img', img)
        k = cv2.waitKey(30) & 0xff
        if k == 27:
            break

    cap.release()
    cv2.destroyAllWindows()


# Check if picture exists
def check_image(image_path):
    if os.path.isfile(image_path):
        return True
    else:
        print('Cannot find image file.')
        return False


# Check if video exists
def check_video(video_cap):
    if video_cap.isOpened():
        return True
    else:
        print('No video stream detected')
        return False


def main():
    # exit all methods with pressing q
    # show_image()
    # show_video()
    # record_video()
    # read_img()
    #detect_faces_from_video()
    #configuration = Configuration(False)
    config = Configuration()


if __name__ == '__main__':
    main()
